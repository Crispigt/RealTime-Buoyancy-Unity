using System.Collections.Generic;
using UnityEngine;

public static class AdaptiveClipper
{
    const float EPS = 1e-4f;

    // Reusable scratch for the wet polygon of the triangle being processed.
    // Static so we don't reallocate per-triangle. Single-threaded use only.
    static readonly List<Vector3> polyVerts = new List<Vector3>(16);
    static readonly List<float> polyHeights = new List<float>(16);

    public static void BuildSubMesh(
        Vector3[] worldVerts, int[] tris, float[] heights,
        WaveManager wm, float t, int refineSamples,
        List<float> outVerts, List<int> outIndices, List<float> outHeights)
    {
        for (int i = 0; i < tris.Length; i += 3)
        {
            int ia = tris[i];
            int ib = tris[i + 1];
            int ic = tris[i + 2];

            Vector3 a = worldVerts[ia];
            Vector3 b = worldVerts[ib];
            Vector3 c = worldVerts[ic];
            float ha = heights[ia];
            float hb = heights[ib];
            float hc = heights[ic];

            // Same classification as the DLL: 1 bit per vertex, set if below water.
            int below = (((a.y - ha) < -EPS) ? 4 : 0)
                      | (((b.y - hb) < -EPS) ? 2 : 0)
                      | (((c.y - hc) < -EPS) ? 1 : 0);

            switch (below)
            {
                case 0b000: continue;
                case 0b111: EmitTriangle(a, b, c, ha, hb, hc, outVerts, outIndices, outHeights); break;
                case 0b100: OneBelow(a, b, c, ha, hb, hc, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
                case 0b010: OneBelow(b, c, a, hb, hc, ha, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
                case 0b001: OneBelow(c, a, b, hc, ha, hb, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
                case 0b011: TwoBelow(a, b, c, ha, hb, hc, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
                case 0b101: TwoBelow(b, c, a, hb, hc, ha, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
                case 0b110: TwoBelow(c, a, b, hc, ha, hb, wm, t, refineSamples, outVerts, outIndices, outHeights); break;
            }
        }
    }

    // (below, above1, above2). Two crossing edges: below->above1 and below->above2.
    // Wet polygon: [below, P1, <refined samples P1->P2>, P2]. Fan from below.
    static void OneBelow(
        Vector3 below, Vector3 first, Vector3 second,
        float hBelow, float hFirst, float hSecond,
        WaveManager wm, float t, int refineSamples,
        List<float> outVerts, List<int> outIndices, List<float> outHeights)
    {
        Vector3 p1; float hp1;
        Vector3 p2; float hp2;
        if (!ClipEdge(below, first, hBelow, hFirst, out p1, out hp1)) return;
        if (!ClipEdge(below, second, hBelow, hSecond, out p2, out hp2)) return;

        polyVerts.Clear();
        polyHeights.Clear();
        polyVerts.Add(below);   polyHeights.Add(hBelow);
        polyVerts.Add(p1);      polyHeights.Add(hp1);
        AppendRefinedSamples(p1, p2, wm, t, refineSamples);
        polyVerts.Add(p2);      polyHeights.Add(hp2);

        FanTriangulate(outVerts, outIndices, outHeights);
    }

    // (above, below1, below2). Two crossing edges: below1->above and below2->above.
    // Wet polygon: [below1, below2, P2, <refined samples P2->P1>, P1]. Fan from below1.
    static void TwoBelow(
        Vector3 above, Vector3 first, Vector3 second,
        float hAbove, float hFirst, float hSecond,
        WaveManager wm, float t, int refineSamples,
        List<float> outVerts, List<int> outIndices, List<float> outHeights)
    {
        Vector3 p1; float hp1;
        Vector3 p2; float hp2;
        if (!ClipEdge(first, above, hFirst, hAbove, out p1, out hp1)) return;
        if (!ClipEdge(second, above, hSecond, hAbove, out p2, out hp2)) return;

        polyVerts.Clear();
        polyHeights.Clear();
        polyVerts.Add(first);   polyHeights.Add(hFirst);
        polyVerts.Add(second);  polyHeights.Add(hSecond);
        polyVerts.Add(p2);      polyHeights.Add(hp2);
        AppendRefinedSamples(p2, p1, wm, t, refineSamples);
        polyVerts.Add(p1);      polyHeights.Add(hp1);

        FanTriangulate(outVerts, outIndices, outHeights);
    }

    // Linear edge clip against per-vertex water heights. Same lerp the DLL uses.
    static bool ClipEdge(
        Vector3 a, Vector3 b, float ha, float hb,
        out Vector3 p, out float hp)
    {
        float da = a.y - ha;
        float db = b.y - hb;
        float denom = da - db;
        if (Mathf.Abs(denom) < 1e-6f) { p = a; hp = ha; return false; }
        float step = Mathf.Clamp01(da / denom);
        p = Vector3.Lerp(a, b, step);
        hp = p.y;  // by construction, on the (linear) waterline
        return true;
    }

    // Sample N evenly-spaced interior points along the chord p1->p2 and append each
    // as a polygon vertex with the TRUE wave height at its xz. The closed-form
    // integrator in the DLL takes a linear water surface across each sub-triangle
    // from these per-vertex heights, so this turns the straight chord-segment of the
    // wet polygon into a piecewise-linear height profile that follows the wave.
    //
    // The asymmetry fix: we add samples unconditionally, not only on sign flips.
    //   * Crest over chord (hr > r.y): sub-triangles near chord get extra depth →
    //     positive correction, capturing the "missing wet area" outside the chord.
    //   * Trough under chord (hr < r.y): sub-triangles near chord get negative depth →
    //     accumulateBuoyancy emits a NEGATIVE contribution, cancelling the spurious
    //     wet area we kept inside the chord. Same closed-form, opposite sign.
    //
    // Skipped silently if wm == null or N == 0 (linear path).
    static void AppendRefinedSamples(
        Vector3 p1, Vector3 p2, WaveManager wm, float t, int refineSamples)
    {
        if (wm == null || refineSamples <= 0) return;

        for (int k = 1; k <= refineSamples; k++)
        {
            float u = (float)k / (refineSamples + 1);
            Vector3 m = Vector3.Lerp(p1, p2, u);
            float hm = wm.SampleHeight(m.x, m.z, t);
            polyVerts.Add(m);
            polyHeights.Add(hm);
        }
    }

    // Fan-triangulate polyVerts/polyHeights into the output buffers, preserving CW winding.
    static void FanTriangulate(
        List<float> outVerts, List<int> outIndices, List<float> outHeights)
    {
        if (polyVerts.Count < 3) return;
        int baseIndex = outHeights.Count;

        for (int i = 0; i < polyVerts.Count; i++)
        {
            outVerts.Add(polyVerts[i].x);
            outVerts.Add(polyVerts[i].y);
            outVerts.Add(polyVerts[i].z);
            outHeights.Add(polyHeights[i]);
        }

        for (int i = 1; i < polyVerts.Count - 1; i++)
        {
            outIndices.Add(baseIndex);
            outIndices.Add(baseIndex + i);
            outIndices.Add(baseIndex + i + 1);
        }
    }

    // Fully submerged passthrough — emit the original triangle into the scratch mesh.
    static void EmitTriangle(
        Vector3 a, Vector3 b, Vector3 c,
        float ha, float hb, float hc,
        List<float> outVerts, List<int> outIndices, List<float> outHeights)
    {
        int baseIndex = outHeights.Count;
        outVerts.Add(a.x); outVerts.Add(a.y); outVerts.Add(a.z); outHeights.Add(ha);
        outVerts.Add(b.x); outVerts.Add(b.y); outVerts.Add(b.z); outHeights.Add(hb);
        outVerts.Add(c.x); outVerts.Add(c.y); outVerts.Add(c.z); outHeights.Add(hc);
        outIndices.Add(baseIndex);
        outIndices.Add(baseIndex + 1);
        outIndices.Add(baseIndex + 2);
    }
}
