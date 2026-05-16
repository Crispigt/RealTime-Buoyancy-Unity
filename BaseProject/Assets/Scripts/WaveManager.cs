using UnityEngine;
using Unity.Mathematics;

// Gerstner wave function adapted from Catlike Coding's "Waves" tutorial
// by Jasper Flick (MIT-0): https://catlikecoding.com/unity/tutorials/flow/waves/
public class WaveManager : MonoBehaviour
{
    public static WaveManager Instance { get; private set; }

    [SerializeField] Material waterMaterial;

    [Tooltip("X, Y direction, Z for steepness, W for wavelength, have to match with the shader, Waves.shader")]
    [SerializeField] Vector4 waveA = new Vector4(1f, 0f,   0.25f, 60f);
    [SerializeField] Vector4 waveB = new Vector4(1f, 0.6f, 0.25f, 31f);
    [SerializeField] Vector4 waveC = new Vector4(1f, 1.3f, 0.25f, 18f);

    [Tooltip("Newton iterations for Gerstner inversion at sample-at-(x,z). 1–2 is sufficient; 4+ gives diminishing returns.")]
    [Range(1, 8)] [SerializeField] int sampleIterations = 4;

    const float G = 9.8f;

    static readonly int WaveAId = Shader.PropertyToID("_WaveA");
    static readonly int WaveBId = Shader.PropertyToID("_WaveB");
    static readonly int WaveCId = Shader.PropertyToID("_WaveC");
    static readonly int WaveTimeId = Shader.PropertyToID("_WaveTime");

    void Awake()
    {
        Instance = this;
        PushToMaterial();
    }

    void OnValidate()
    {
        if (waterMaterial != null) PushToMaterial();
    }

    void PushToMaterial()
    {
        waterMaterial.SetVector(WaveAId, waveA);
        waterMaterial.SetVector(WaveBId, waveB);
        waterMaterial.SetVector(WaveCId, waveC);
        waterMaterial.SetFloat(WaveTimeId, Application.isPlaying ? Time.fixedTime : 0f);
    }

    void FixedUpdate()
    {
        if (waterMaterial != null)
        {
            waterMaterial.SetFloat(WaveTimeId, Time.fixedTime);
        }
    }

    // Snapshot for the Burst job — plain values only, safe to copy into a struct.
    public void GetWaveParams(out float4 a, out float4 b, out float4 c, out int iters)
    {
        a = new float4(waveA.x, waveA.y, waveA.z, waveA.w);
        b = new float4(waveB.x, waveB.y, waveB.z, waveB.w);
        c = new float4(waveC.x, waveC.y, waveC.z, waveC.w);
        iters = sampleIterations;
    }

    // Single-wave displacement at rest position p (y ignored). Same as shader.
    static Vector3 GerstnerDisp(Vector4 wave, float px, float pz, float t)
    {
        float steepness = wave.z;
        float wavelength = wave.w;
        float k = 2f * Mathf.PI / wavelength;
        float c = Mathf.Sqrt(G / k);
        Vector2 d = new Vector2(wave.x, wave.y).normalized;
        float f = k * (d.x * px + d.y * pz - c * t);
        float a = steepness / k;
        return new Vector3(d.x * (a * Mathf.Cos(f)),
                                   a * Mathf.Sin(f),
                           d.y * (a * Mathf.Cos(f)));
    }

    static Vector3 GerstnerDispAndJacobian(
    Vector4 wave,
    float px,
    float pz,
    float t,
    ref float j00,
    ref float j01,
    ref float j10,
    ref float j11)
    {
        float steepness = wave.z;
        float wavelength = wave.w;
        float k = 2f * Mathf.PI / wavelength;
        float c = Mathf.Sqrt(G / k);
        Vector2 d = new Vector2(wave.x, wave.y).normalized;
        float f = k * (d.x * px + d.y * pz - c * t);
        float sinF = Mathf.Sin(f);
        float cosF = Mathf.Cos(f);
        float a = steepness / k;

        float steepSin = steepness * sinF;
        j00 -= d.x * d.x * steepSin;
        j01 -= d.x * d.y * steepSin;
        j10 -= d.y * d.x * steepSin;
        j11 -= d.y * d.y * steepSin;

        return new Vector3(
            d.x * (a * cosF),
            a * sinF,
            d.y * (a * cosF));
    }

    // Returns surface height y at world (x, z) at time t.
    // Inverts the Gerstner mapping by Newton iteration (2×2 Jacobian solve) so the result is the
    // height above (x, z), not the height of the particle that originated at (x, z).
    public float SampleHeight(float x, float z, float t)
    {
        float px = x, pz = z;

        for (int i = 0; i < sampleIterations; i++)
        {
            float j00 = 1f, j01 = 0f, j10 = 0f, j11 = 1f;

            Vector3 disp = GerstnerDispAndJacobian(waveA, px, pz, t, ref j00, ref j01, ref j10, ref j11)
                         + GerstnerDispAndJacobian(waveB, px, pz, t, ref j00, ref j01, ref j10, ref j11)
                         + GerstnerDispAndJacobian(waveC, px, pz, t, ref j00, ref j01, ref j10, ref j11);

            float fx = px + disp.x - x;
            float fz = pz + disp.z - z;

            float det = j00 * j11 - j01 * j10;
            if (Mathf.Abs(det) < 1e-5f) break;

            float dx = (fx * j11 - j01 * fz) / det;
            float dz = (j00 * fz - fx * j10) / det;

            px -= dx;
            pz -= dz;

            if (dx * dx + dz * dz < 1e-6f) break;
        }

        Vector3 finalDisp = GerstnerDisp(waveA, px, pz, t)
                          + GerstnerDisp(waveB, px, pz, t)
                          + GerstnerDisp(waveC, px, pz, t);
        return finalDisp.y;
    }
}
