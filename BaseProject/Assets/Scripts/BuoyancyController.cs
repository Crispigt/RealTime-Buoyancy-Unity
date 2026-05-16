using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using UnityEngine.UIElements;

public class BuoyancyController : MonoBehaviour
{
    [Header("Mesh")]
    [SerializeField] MeshFilter buoyancyMesh;

    [Tooltip("Enable for Adaptive path: C# refines the waterline and triangulates the wet region; DLL acts as a pure integrator.\n\nDisable for Linear path: Cheaper. C# samples per vertex and DLL handles clipping. Best when edges are short vs wavelength.")]
    [SerializeField] bool adaptiveClipping = false;
    [SerializeField] int refineSamples;


    [Header("Enviorment")]
    [SerializeField] float rho = 1000f;
    [SerializeField] float gravity = 9.81f;
    [Header("Damping")]
    [SerializeField, Range(0.9f, 1.0f)] float angularAlpha = 0.98f;
    [SerializeField] float linearDrag = 3f;  // drag force when in water (0 = no drag)

    [Header("Mesh simplification")]
    [SerializeField, Range(10, 100000)] int targetBuoyancyTriangles = 1000;

    [Header("Debug")]
    [SerializeField] bool showDebug = false;
    [SerializeField] float normalLength = 0.15f;
    [SerializeField] float forceScale = 0.01f;

    [Header("Validation")]
    [SerializeField] bool logEquilibrium = false;

    private int frameCount;
    private Rigidbody rb;
    private int handle;
    float[] res = new float[6];
    /*[SerializeField]*/ Vector3 lastForce;
    /*[SerializeField]*/ Vector3 lastTorque;
    Vector3 L;  // angular momentum

    // Pre-allocated to avoid per-frame GC.
    Vector3[] localVerts;
    int[] localTris;
    float[] vertexHeights;
    float[] matrix = new float[16];
    Transform meshTransform;  // may differ from this.transform when mesh is on a child
    
    private List<float> scratchVerts;
    private List<int> scratchIndices;
    private List<float> scratchHeights;

    float[] scratchVertsArr;
    int[] scratchIndicesArr;
    float[] scratchHeightsArr;

    Vector3[] worldVerts;

    // Burst job storage, persistent for the lifetime of the body.
    NativeArray<float3> localVertsNA;
    NativeArray<float3> worldVertsNA;
    NativeArray<float> heightsNA;

    void Start()
    {
        // Get all data needed for calculations
        rb = GetComponent<Rigidbody>();

        // Init angular momentum from current angular velocity
        Quaternion q = rb.inertiaTensorRotation;
        Vector3 w = rb.angularVelocity;
        L = q * Vector3.Scale(rb.inertiaTensor, Quaternion.Inverse(q) * w);

        MeshFilter mf = (buoyancyMesh == null) ? GetComponentInChildren<MeshFilter>() : buoyancyMesh;
        meshTransform = mf.transform;
        Mesh mesh = mf.sharedMesh;

        int originalTriangleCount = mesh.triangles.Length / 3;
        int targetTriangleCount = Mathf.Clamp(targetBuoyancyTriangles, 4, originalTriangleCount);
        float quality = targetTriangleCount / (float)originalTriangleCount;

        if (quality < 0.999f)
        {
            var simplifier = new UnityMeshSimplifier.MeshSimplifier();
            simplifier.Initialize(mesh);
            simplifier.SimplifyMesh(quality);
            mesh = simplifier.ToMesh();

            //Debug.Log($"{name}: simplified buoyancy mesh from {originalTriangleCount} to {mesh.triangles.Length / 3} tris");
        }

        Vector3[] verts = mesh.vertices;
        int[] tris = mesh.triangles;
        localVerts = verts;
        localTris = tris;
        vertexHeights = new float[verts.Length];
        float[] vertFloats = new float[verts.Length * 3];
        for (int i = 0; i < verts.Length; i++)
        {
            vertFloats[i * 3] = verts[i].x;
            vertFloats[i * 3 + 1] = verts[i].y;
            vertFloats[i * 3 + 2] = verts[i].z;
        }
        Vector3 com = rb.centerOfMass;
        // Send to C++ script
        handle = NativeBridge.CreateBuoyancyInstance(
            vertFloats,
            verts.Length,
            tris,
            tris.Length,
            com.x,
            com.y,
            com.z,
            rho,
            gravity
            );

        scratchVerts = new List<float>();
        scratchIndices = new List<int>();
        scratchHeights = new List<float>();
        worldVerts = new Vector3[verts.Length];

        // Burst job buffers. localVerts is filled once and never mutated;
        // worldVerts and heights are written each FixedUpdate.
        localVertsNA = new NativeArray<float3>(verts.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        worldVertsNA = new NativeArray<float3>(verts.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        heightsNA    = new NativeArray<float>(verts.Length, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        for (int i = 0; i < verts.Length; i++) localVertsNA[i] = verts[i];
    }

    // Update is 50hz
    void FixedUpdate()
    {
        //float depth = -transform.position.y;  // positive below y=0
        //if (depth > 0)
        //{
        //    float force = rho * 9.81f * depth * area;
        //    rb.AddForce(Vector3.up * force, ForceMode.Force);
        //}

        Matrix4x4 m = meshTransform.localToWorldMatrix;
        for (int i = 0; i < 16; i++)
        {
            matrix[i] = m[i];
        }

        float dt = Time.fixedDeltaTime;
        float t = Time.fixedTime;
        WaveManager wm = WaveManager.Instance;

        Profiler.BeginSample("Buoy.WaveSampleJob");
        if (wm != null)
        {
            wm.GetWaveParams(out float4 wa, out float4 wb, out float4 wc, out int iters);
            var job = new WaveSampleJob
            {
                localVerts = localVertsNA,
                matrix = m,
                waveA = wa, waveB = wb, waveC = wc,
                t = t,
                iterations = iters,
                worldVerts = worldVertsNA,
                heights = heightsNA,
            };
            
            job.Schedule(localVertsNA.Length, 32).Complete();

            // Copy back to the managed arrays the rest of the pipeline (DLL marshal,
            // AdaptiveClipper, gizmos) consumes.
            for (int i = 0; i < localVerts.Length; i++)
            {
                float3 wv = worldVertsNA[i];
                worldVerts[i] = new Vector3(wv.x, wv.y, wv.z);
                vertexHeights[i] = heightsNA[i];
            }
        }
        else
        {
            for (int i = 0; i < localVerts.Length; i++)
            {
                worldVerts[i] = m.MultiplyPoint3x4(localVerts[i]);
                vertexHeights[i] = 0f;
            }
        }
        Profiler.EndSample();

        if (adaptiveClipping)
        {
            scratchVerts.Clear();
            scratchIndices.Clear();
            scratchHeights.Clear();

            AdaptiveClipper.BuildSubMesh(
                worldVerts, localTris, vertexHeights,
                wm, t, refineSamples,
                scratchVerts, scratchIndices, scratchHeights);

            int vc = scratchVerts.Count;
            int ic = scratchIndices.Count;
            if (scratchVertsArr == null || scratchVertsArr.Length < vc) scratchVertsArr = new float[vc];
            if (scratchIndicesArr == null || scratchIndicesArr.Length < ic) scratchIndicesArr = new int[ic];
            if (scratchHeightsArr == null || scratchHeightsArr.Length < scratchHeights.Count) scratchHeightsArr = new float[scratchHeights.Count];

            scratchVerts.CopyTo(scratchVertsArr);
            scratchIndices.CopyTo(scratchIndicesArr);
            scratchHeights.CopyTo(scratchHeightsArr);

            Vector3 comW = transform.TransformPoint(rb.centerOfMass);
            NativeBridge.ComputeBuoyancyFromTriangles(
                handle, scratchVertsArr, scratchIndicesArr, ic,
                scratchHeightsArr, res,
                comW.x, comW.y, comW.z);

        }
        else
        {
            NativeBridge.ComputeBuoyancy(handle, matrix, vertexHeights, vertexHeights.Length, res);
        }

        lastForce = new Vector3(res[0], res[1], res[2]);
        lastTorque = new Vector3(res[3], res[4], res[5]);

        if (!rb.isKinematic)
        {
            // Linear dampning, unity handles gravity. We apply buoyancy + water drag.
            rb.AddForce(lastForce, ForceMode.Force);
            if (lastForce.sqrMagnitude > 0.0001f)  // in water
                rb.AddForce(-linearDrag * rb.linearVelocity, ForceMode.Acceleration);

            // Angular dampning, integrate torque -> momentum -> damp -> velocity
            L += lastTorque * dt;
            L *= angularAlpha;
            Quaternion q = rb.inertiaTensorRotation;
            Vector3 L_local = Quaternion.Inverse(q) * L;
            Vector3 omega_local = Vector3.zero;
            if (rb.inertiaTensor.x > 1e-6f) omega_local.x = L_local.x / rb.inertiaTensor.x;
            if (rb.inertiaTensor.y > 1e-6f) omega_local.y = L_local.y / rb.inertiaTensor.y;
            if (rb.inertiaTensor.z > 1e-6f) omega_local.z = L_local.z / rb.inertiaTensor.z;
            
            Vector3 finalAngularVelocity = q * omega_local;
            if (!float.IsNaN(finalAngularVelocity.x) && !float.IsNaN(finalAngularVelocity.y) && !float.IsNaN(finalAngularVelocity.z))
            {
                rb.angularVelocity = finalAngularVelocity;
            }
        }
        else
        {
            L = Vector3.zero;
        }

        //if (Time.frameCount % 50 == 0)
        //{
        //    Vector3 worldOrigin = transform.position;
        //    Debug.Log($"Interia: {rb.inertiaTensor:F2}  " +
        //      $"Com: {rb.centerOfMass:F2}  " +
        //      $"lastTorque: {lastTorque.magnitude:F2}  ");
        //}
    }





    private void Update()
    {
        if (showDebug)
        {
            Debug.DrawRay(transform.position, lastForce * forceScale, Color.red);
            Debug.DrawRay(transform.position, lastTorque * forceScale, Color.blue);
        }
        if (logEquilibrium && ++frameCount % 50 == 0)
        {
            float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
            Debug.Log($"Tilt: {tiltAngle:F2}° | "
                    + $"|F_b|: {lastForce.magnitude:F2} (mg: {rb.mass * Physics.gravity.magnitude:F2}) | "
                    + $"|τ|: {lastTorque.magnitude:F6}");
        }
    }

    //Want no mem leaks
    private void OnDestroy()
    {
        NativeBridge.DestroyBuoyancyInstance(handle);
        if (localVertsNA.IsCreated) localVertsNA.Dispose();
        if (worldVertsNA.IsCreated) worldVertsNA.Dispose();
        if (heightsNA.IsCreated) heightsNA.Dispose();
    }

    // For debug visuals
    void OnDrawGizmos()
    {
        if (!showDebug) return;

        Vector3[] verts;
        int[] tris;
        if (Application.isPlaying && localVerts != null && localTris != null)
        {
            verts = localVerts;
            tris = localTris;
        }
        else
        {
            Mesh mesh = GetComponentInChildren<MeshFilter>()?.sharedMesh;
            if (mesh == null) return;
            verts = mesh.vertices;
            tris = mesh.triangles;
        }

        Transform tf = (Application.isPlaying && meshTransform != null) ? meshTransform : transform;
        WaveManager wmGizmo = WaveManager.Instance;
        float gizmoTime = Application.isPlaying ? Time.fixedTime : 0f;

        for (int i = 0; i < tris.Length; i += 3)
        {
            int ia = tris[i];
            int ib = tris[i + 1];
            int ic = tris[i + 2];

            Vector3 a = tf.TransformPoint(verts[ia]);
            Vector3 b = tf.TransformPoint(verts[ib]);
            Vector3 c = tf.TransformPoint(verts[ic]);

            float ha, hb, hc;
            if (Application.isPlaying && vertexHeights != null && vertexHeights.Length == verts.Length)
            {
                ha = vertexHeights[ia];
                hb = vertexHeights[ib];
                hc = vertexHeights[ic];
            }
            else
            {
                ha = wmGizmo != null ? wmGizmo.SampleHeight(a.x, a.z, gizmoTime) : 0f;
                hb = wmGizmo != null ? wmGizmo.SampleHeight(b.x, b.z, gizmoTime) : 0f;
                hc = wmGizmo != null ? wmGizmo.SampleHeight(c.x, c.z, gizmoTime) : 0f;
            }

            bool aBelow = a.y < ha, bBelow = b.y < hb, cBelow = c.y < hc;
            int belowCount = (aBelow ? 1 : 0) + (bBelow ? 1 : 0) + (cBelow ? 1 : 0);

            Color triColor;
            if (belowCount == 3) triColor = Color.blue;   // fully submerged
            else if (belowCount == 0) triColor = Color.green;  // fully dry
            else triColor = new Color(1f, 0.5f, 0f); // orange = intersecting

            // Wireframe
            Gizmos.color = triColor;
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, a);

            // Clip-point visualization for intersecting triangles 
            if (belowCount == 1 || belowCount == 2)
            {
                DrawClipVisualization(a, b, c, ha, hb, hc, aBelow, bBelow, cBelow, wmGizmo, gizmoTime);
            }
        }
    }


    void DrawClipVisualization(
        Vector3 a, Vector3 b, Vector3 c,
        float ha, float hb, float hc,
        bool aBelow, bool bBelow, bool cBelow,
        WaveManager wm, float time)
    {
        // Identify the below / above vertices using the same cyclic-permutation
        // convention as the DLL and AdaptiveClipper.
        Vector3 p1, p2;
        float hp1, hp2;
        // Track which vertices are below for sub-triangle drawing.
        Vector3 belowVert1 = Vector3.zero, belowVert2 = Vector3.zero;
        bool isOneBelow;

        if (aBelow && !bBelow && !cBelow) // one below: a
        {
            if (!TryClipEdge(a, b, ha, hb, out p1, out hp1)) return;
            if (!TryClipEdge(a, c, ha, hc, out p2, out hp2)) return;
            belowVert1 = a; isOneBelow = true;
        }
        else if (!aBelow && bBelow && !cBelow) // one below: b
        {
            if (!TryClipEdge(b, c, hb, hc, out p1, out hp1)) return;
            if (!TryClipEdge(b, a, hb, ha, out p2, out hp2)) return;
            belowVert1 = b; isOneBelow = true;
        }
        else if (!aBelow && !bBelow && cBelow) // one below: c
        {
            if (!TryClipEdge(c, a, hc, ha, out p1, out hp1)) return;
            if (!TryClipEdge(c, b, hc, hb, out p2, out hp2)) return;
            belowVert1 = c; isOneBelow = true;
        }
        else if (!aBelow && bBelow && cBelow) // two below: b,c — above: a
        {
            if (!TryClipEdge(b, a, hb, ha, out p1, out hp1)) return;
            if (!TryClipEdge(c, a, hc, ha, out p2, out hp2)) return;
            belowVert1 = b; belowVert2 = c; isOneBelow = false;
        }
        else if (aBelow && !bBelow && cBelow) // two below: a,c — above: b
        {
            if (!TryClipEdge(a, b, ha, hb, out p1, out hp1)) return;
            if (!TryClipEdge(c, b, hc, hb, out p2, out hp2)) return;
            belowVert1 = a; belowVert2 = c; isOneBelow = false;
        }
        else if (aBelow && bBelow && !cBelow) // two below: a,b — above: c
        {
            if (!TryClipEdge(a, c, ha, hc, out p1, out hp1)) return;
            if (!TryClipEdge(b, c, hb, hc, out p2, out hp2)) return;
            belowVert1 = a; belowVert2 = b; isOneBelow = false;
        }
        else return;

        // Draw clip-point spheres (cyan)
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(p1, 0.02f);
        Gizmos.DrawSphere(p2, 0.02f);

        // Draw the linear waterline chord (white)
        Gizmos.color = Color.white;
        Gizmos.DrawLine(p1, p2);

        Color subTriColor = new Color(1f, 0.2f, 0.2f, 1f); // red for sub-triangles
        Gizmos.color = subTriColor;

        if (!(adaptiveClipping && wm != null && refineSamples > 0))
        {
            // Linear path draw the simple clipped sub-triangle(s)
            if (isOneBelow)
            {
                // One sub-triangle [below, p1, p2]
                DrawTriangleWireframe(belowVert1, p1, p2);
            }
            else
            {
                // Quad split into two sub-triangles: [b1, b2, p1] and [b2, p2, p1]
                DrawTriangleWireframe(belowVert1, belowVert2, p1);
                DrawTriangleWireframe(belowVert2, p2, p1);
            }
        }
        else
        {
            // Adaptive path build the polyline, then draw fan sub-triangles
            // Build the polygon vertices along the chord (same as AdaptiveClipper)
            var polyline = new System.Collections.Generic.List<Vector3>();
            var waveHeights = new System.Collections.Generic.List<float>();

            polyline.Add(p1);
            waveHeights.Add(p1.y);

            for (int k = 1; k <= refineSamples; k++)
            {
                float u = (float)k / (refineSamples + 1);
                Vector3 m = Vector3.Lerp(p1, p2, u);
                polyline.Add(m);
                
                float hm = wm != null ? wm.SampleHeight(m.x, m.z, time) : m.y;
                waveHeights.Add(hm);
            }
            polyline.Add(p2);
            waveHeights.Add(p2.y);

            // Draw the adaptive polyline (the chord) in magenta
            for (int k = 0; k < polyline.Count - 1; k++)
            {
                Gizmos.color = Color.magenta;
                Gizmos.DrawLine(polyline[k], polyline[k + 1]);
                
                if (k > 0) // skip p1
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawSphere(polyline[k], 0.015f);

                    // Draw a vertical line from the chord to the actual wave height
                    Gizmos.color = Color.green;
                    Vector3 waveSurface = new Vector3(polyline[k].x, waveHeights[k], polyline[k].z);
                    Gizmos.DrawLine(polyline[k], waveSurface);
                    Gizmos.DrawSphere(waveSurface, 0.01f);
                }
            }

            // Draw the fan-triangulated sub-triangles
            Gizmos.color = subTriColor;
            if (isOneBelow)
            {
                // Fan from belowVert1 through the polyline
                for (int k = 0; k < polyline.Count - 1; k++)
                    DrawTriangleWireframe(belowVert1, polyline[k], polyline[k + 1]);
            }
            else
            {
                // Polygon: [b1, b2, p2, ..refined.., p1] — fan from b1
                var fullPoly = new System.Collections.Generic.List<Vector3>();
                fullPoly.Add(belowVert1);
                fullPoly.Add(belowVert2);
                fullPoly.Add(p2);
                // Add refined samples in reverse (p2 → p1 direction)
                for (int k = refineSamples; k >= 1; k--)
                {
                    float u = (float)k / (refineSamples + 1);
                    fullPoly.Add(Vector3.Lerp(p1, p2, u));
                }
                fullPoly.Add(p1);

                for (int k = 1; k < fullPoly.Count - 1; k++)
                    DrawTriangleWireframe(fullPoly[0], fullPoly[k], fullPoly[k + 1]);
            }
        }
    }

    static void DrawTriangleWireframe(Vector3 v0, Vector3 v1, Vector3 v2)
    {
        Gizmos.DrawLine(v0, v1);
        Gizmos.DrawLine(v1, v2);
        Gizmos.DrawLine(v2, v0);
    }

    static bool TryClipEdge(Vector3 below, Vector3 above, float hBelow, float hAbove,
        out Vector3 p, out float hp)
    {
        float dBelow = below.y - hBelow;
        float dAbove = above.y - hAbove;
        float denom = dAbove - dBelow;
        if (Mathf.Abs(denom) < 1e-6f) { p = below; hp = hBelow; return false; }
        float t = dAbove / denom;
        t = Mathf.Clamp01(t);
        p = Vector3.Lerp(above, below, t);
        hp = Mathf.Lerp(hAbove, hBelow, t);
        return true;
    }
}
