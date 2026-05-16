using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

// Burst-compiled per-vertex wave sampling. Mirrors WaveManager.SampleHeight exactly,
// but operates on plain math types (no managed Vector3, no MonoBehaviour access) so
// Burst can vectorise/inline the inner Newton iteration over the Gerstner inverse.
//
// Also folds in the world-transform of the local mesh vertex, since it's free here:
// one matrix-multiply per vertex. That's the same loop BuoyancyController used to do
// in managed code, so we save a second pass over the vertex array too.
[BurstCompile(FloatPrecision = FloatPrecision.Standard, FloatMode = FloatMode.Fast)]
public struct WaveSampleJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> localVerts;
    public float4x4 matrix;

    public float4 waveA;
    public float4 waveB;
    public float4 waveC;
    public float t;
    public int iterations;

    [WriteOnly] public NativeArray<float3> worldVerts;
    [WriteOnly] public NativeArray<float> heights;

    const float G = 9.8f;

    public void Execute(int i)
    {
        float3 lv = localVerts[i];
        float3 wv = math.transform(matrix, lv);
        worldVerts[i] = wv;
        heights[i] = SampleHeight(wv.x, wv.z);
    }

    float SampleHeight(float x, float z)
    {
        float px = x, pz = z;

        for (int i = 0; i < iterations; i++)
        {
            float j00 = 1f, j01 = 0f, j10 = 0f, j11 = 1f;

            float3 disp = GerstnerDispJ(waveA, px, pz, ref j00, ref j01, ref j10, ref j11)
                        + GerstnerDispJ(waveB, px, pz, ref j00, ref j01, ref j10, ref j11)
                        + GerstnerDispJ(waveC, px, pz, ref j00, ref j01, ref j10, ref j11);

            float fx = px + disp.x - x;
            float fz = pz + disp.z - z;

            float det = j00 * j11 - j01 * j10;
            if (math.abs(det) < 1e-5f) break;

            float dx = (fx * j11 - j01 * fz) / det;
            float dz = (j00 * fz - fx * j10) / det;

            px -= dx;
            pz -= dz;

            if (dx * dx + dz * dz < 1e-6f) break;
        }

        // Final y from the summed Gerstner displacement at the converged rest position.
        return GerstnerY(waveA, px, pz)
             + GerstnerY(waveB, px, pz)
             + GerstnerY(waveC, px, pz);
    }

    float3 GerstnerDispJ(float4 wave, float px, float pz,
        ref float j00, ref float j01, ref float j10, ref float j11)
    {
        float steepness = wave.z;
        float wavelength = wave.w;
        float k = 2f * math.PI / wavelength;
        float c = math.sqrt(G / k);
        float2 d = math.normalize(new float2(wave.x, wave.y));
        float f = k * (d.x * px + d.y * pz - c * t);
        math.sincos(f, out float sinF, out float cosF);
        float a = steepness / k;

        float steepSin = steepness * sinF;
        j00 -= d.x * d.x * steepSin;
        j01 -= d.x * d.y * steepSin;
        j10 -= d.y * d.x * steepSin;
        j11 -= d.y * d.y * steepSin;

        return new float3(d.x * (a * cosF), a * sinF, d.y * (a * cosF));
    }

    float GerstnerY(float4 wave, float px, float pz)
    {
        float steepness = wave.z;
        float wavelength = wave.w;
        float k = 2f * math.PI / wavelength;
        float c = math.sqrt(G / k);
        float2 d = math.normalize(new float2(wave.x, wave.y));
        float f = k * (d.x * px + d.y * pz - c * t);
        return (steepness / k) * math.sin(f);
    }
}
