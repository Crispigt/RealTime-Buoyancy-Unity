using System.Collections;
using System.Text;
using UnityEngine;

/// <summary>
/// Accuracy test runner for the buoyancy simulation.
/// Attach to an empty GameObject in a dedicated test scene.
/// Each test mode is selected from the inspector; hit Play to run.
///
/// OUTPUT: CSV-formatted lines prefixed with [TEST] — copy from
/// the Unity console and paste into Excel / Python for plotting.
/// </summary>
public class AccuracyTests : MonoBehaviour
{
    public enum TestMode
    {
        A1_CubeSubmersionSweep,
        A2_SphereAnalytical,
        A3_EquilibriumTilt,
        A5_WaveZeroRegression,
        A6_AdaptiveVsLinearAB,
    }

    [Header("Test Selection")]
    [SerializeField] TestMode mode = TestMode.A1_CubeSubmersionSweep;

    [Header("Target Body")]
    [Tooltip("The BuoyancyController on the cube / sphere / bunny to test.")]
    [SerializeField] BuoyancyController target;
    [SerializeField] Rigidbody targetRb;

    // ── A1 / A2 settings ──────────────────────────────────────────────────
    [Header("Submersion Sweep (A1 / A2)")]
    [Tooltip("Water surface y when all wave amplitudes are 0.")]
    [SerializeField] float waterY = 0f;
    [Tooltip("Body starts at this height above water.")]
    [SerializeField] float sweepStart = 0.6f;
    [Tooltip("Body ends at this depth below water (negative = submerged).")]
    [SerializeField] float sweepEnd = -0.6f;
    [SerializeField] int sweepSteps = 25;
    [Tooltip("Frames to wait after teleporting before reading force (let physics settle).")]
    [SerializeField] int settleFrames = 5;

    // ── A3 settings ────────────────────────────────────────────────────────
    [Header("Equilibrium Tilt (A3)")]
    [Tooltip("How many FixedUpdates to wait before sampling the settled state.")]
    [SerializeField] int settleTime = 500;  // ~10 s at 50 Hz
    [Tooltip("Constrain rotation to one axis only (matches Hirae 2D-bar setup).")]
    [SerializeField] bool constrainToOneAxis = true;

    // ── A6 settings ────────────────────────────────────────────────────────
    [Header("Adaptive vs Linear A/B (A6)")]
    [Tooltip("A second BuoyancyController with adaptiveClipping=true (e.g. N=4).")]
    [SerializeField] BuoyancyController adaptiveBody;
    [SerializeField] Rigidbody adaptiveRb;
    
    [Tooltip("A third BuoyancyController with adaptiveClipping=true (e.g. N=8) to test convergence.")]
    [SerializeField] BuoyancyController adaptiveBody2;
    [SerializeField] Rigidbody adaptiveRb2;
    
    [Tooltip("A fourth BuoyancyController using a highly dense mesh (e.g. 30k tris) as ground truth.")]
    [SerializeField] BuoyancyController denseBody;
    [SerializeField] Rigidbody denseRb;
    
    [SerializeField] int abDurationFrames = 300;  // ~6 s

    // ── internals ──────────────────────────────────────────────────────────
    bool _ran = false;
    readonly StringBuilder _csv = new StringBuilder();

    void Start()
    {
        if (target == null || targetRb == null)
        {
            Debug.LogError("[AccuracyTests] Assign Target and TargetRb in the inspector.");
            return;
        }

        // Freeze waves for all static tests.
        if (WaveManager.Instance != null && mode != TestMode.A5_WaveZeroRegression)
            Debug.LogWarning("[AccuracyTests] Set all wave steepnesses to 0 in WaveManager for tests A1–A4.");

        StartCoroutine(RunTest());
    }

    IEnumerator RunTest()
    {
        // Wait one frame for everything to Start().
        yield return null;

        switch (mode)
        {
            case TestMode.A1_CubeSubmersionSweep:   yield return RunA1(); break;
            case TestMode.A2_SphereAnalytical:       yield return RunA2(); break;
            case TestMode.A3_EquilibriumTilt:        yield return RunA3(); break;
            case TestMode.A5_WaveZeroRegression:     yield return RunA5(); break;
            case TestMode.A6_AdaptiveVsLinearAB:     yield return RunA6(); break;
        }

        Debug.Log($"[TEST] ===== {mode} COMPLETE =====");
    }

    // ── A1: Cube submersion force sweep ───────────────────────────────────
    // For a 1×1×1 cube with density ρ_cube, the analytical buoyant force at
    // a given water height h (measured from cube center) is:
    //   submerged_fraction = clamp(0.5 - h, 0, 1)
    //   F_analytical = ρ_water * g * submerged_fraction * Volume
    // where Volume = side³ = 1 m³ for a unit cube.
    IEnumerator RunA1()
    {
        float g = Physics.gravity.magnitude;
        float rho = 1000f;  // water density — must match BuoyancyController.rho
        float side = 1f;    // cube side length in metres — adjust if your cube is scaled

        // Disable physics so we can teleport cleanly.
        FreezeBody(targetRb);

        _csv.AppendLine("// A1 Cube Submersion Sweep");
        _csv.AppendLine("// height_above_water_m, F_computed_N, F_analytical_N, error_N");
        Debug.Log("[TEST] A1 header: height_above_water_m, F_computed_N, F_analytical_N, error_N");

        for (int step = 0; step <= sweepSteps; step++)
        {
            float t = step / (float)sweepSteps;
            float heightAboveWater = Mathf.Lerp(sweepStart, sweepEnd, t);

            // Teleport: cube center at (waterY + heightAboveWater)
            targetRb.position = new Vector3(0f, waterY + heightAboveWater, 0f);
            targetRb.rotation = Quaternion.identity;

            // Let BuoyancyController run for a few FixedUpdates to compute a fresh force.
            for (int f = 0; f < settleFrames; f++)
                yield return new WaitForFixedUpdate();

            // Read the last computed force from BuoyancyController.
            Vector3 computed = GetLastForce(target);
            float Fcomputed = computed.y;  // upward component

            // Analytical: clamp submerged height to [0, side].
            float submergedDepth = Mathf.Clamp(-heightAboveWater + side * 0.5f, 0f, side);
            float Fanalytical = rho * g * submergedDepth * side * side;

            float error = Fcomputed - Fanalytical;

            string line = $"{heightAboveWater:F4}, {Fcomputed:F4}, {Fanalytical:F4}, {error:F4}";
            _csv.AppendLine(line);
            Debug.Log("[TEST] " + line);
        }

        UnfreezeBody(targetRb);
        Debug.Log("[TEST]\n" + _csv.ToString());
    }

    // ── A2: Sphere analytical comparison ─────────────────────────────────
    // For a unit sphere (radius=1, center at origin), buoyant force when
    // submerged to height h above center:
    //   F(h) = (π/3) * ρ * g * (h - 1)² * (h + 2)   [h ∈ (-1, 1)]
    // (This is the standard spherical cap volume formula.)
    IEnumerator RunA2()
    {
        float g = Physics.gravity.magnitude;
        float rho = 1000f;
        float radius = 0.5f;  // adjust to match your sphere's actual radius

        FreezeBody(targetRb);

        _csv.AppendLine("// A2 Sphere Analytical Comparison");
        _csv.AppendLine("// height_above_water_m, F_computed_N, F_analytical_N, error_N");
        Debug.Log("[TEST] A2 header: height_above_water_m, F_computed_N, F_analytical_N, error_N");

        for (int step = 0; step <= sweepSteps; step++)
        {
            float t = step / (float)sweepSteps;
            float heightAboveWater = Mathf.Lerp(sweepStart * radius, sweepEnd * radius, t);

            targetRb.position = new Vector3(0f, waterY + heightAboveWater, 0f);
            targetRb.rotation = Quaternion.identity;

            for (int f = 0; f < settleFrames; f++)
                yield return new WaitForFixedUpdate();

            Vector3 computed = GetLastForce(target);
            float Fcomputed = computed.y;

            // Analytical spherical cap formula — h here is signed distance of center above surface
            // (positive = center above water, negative = center below)
            float h = heightAboveWater;           // center y relative to water
            float clampedH = Mathf.Clamp(h, -radius, radius);  // clip to sphere extent
            float Fanalytical = 0f;
            if (clampedH < radius)  // at least partially submerged
            {
                // Volume of spherical cap submerged = π/3 * (R-h)² * (2R+h)
                // where R = radius, h = signed height of water plane above center
                float waterAboveCenter = -clampedH;  // positive when center below water
                float cap = waterAboveCenter + radius; // cap height (from bottom of sphere to water)
                if (cap > 0f)
                    Fanalytical = (Mathf.PI / 3f) * rho * g * cap * cap * (3f * radius - cap);
            }

            float error = Fcomputed - Fanalytical;
            string line = $"{heightAboveWater:F4}, {Fcomputed:F4}, {Fanalytical:F4}, {error:F4}";
            _csv.AppendLine(line);
            Debug.Log("[TEST] " + line);
        }

        UnfreezeBody(targetRb);
        Debug.Log("[TEST]\n" + _csv.ToString());
    }

    // ── A3: Equilibrium tilt (Hirae Table 1) ─────────────────────────────
    // Drop the cube with density ratio 0.75, constrained to one rotation axis.
    // Expected settled tilt: 26.565° ± 0.01°.  Residual torque: ~1e-4 N·m.
    IEnumerator RunA3()
    {
        // Constrain to rotation around Z only (matches Hirae's 2D bar setup).
        if (constrainToOneAxis)
        {
            targetRb.constraints = RigidbodyConstraints.FreezeRotationX
                                 | RigidbodyConstraints.FreezeRotationY
                                 | RigidbodyConstraints.FreezePositionX
                                 | RigidbodyConstraints.FreezePositionZ;
        }

        // Drop from a slight tilt so it has a reason to rotate.
        targetRb.position = new Vector3(0f, 0.1f, 0f);
        targetRb.rotation = Quaternion.Euler(0f, 0f, 5f);  // small initial tilt
        targetRb.linearVelocity = Vector3.zero;
        targetRb.angularVelocity = Vector3.zero;

        Debug.Log("[TEST] A3: Running for " + settleTime + " FixedUpdates (~" + (settleTime / 50f) + "s). Waiting...");

        _csv.AppendLine("// A3 Equilibrium Tilt");
        _csv.AppendLine("// frame, tilt_deg, Fb_N, mg_N, residual_torque_Nm");
        Debug.Log("[TEST] A3 header: frame, tilt_deg, Fb_N, mg_N, residual_torque_Nm");

        for (int frame = 0; frame < settleTime; frame++)
        {
            yield return new WaitForFixedUpdate();

            // Log every 50 frames (~1 s).
            if (frame % 50 == 0)
            {
                float tilt = Vector3.Angle(targetRb.transform.up, Vector3.up);
                Vector3 force = GetLastForce(target);
                Vector3 torque = GetLastTorque(target);
                float mg = targetRb.mass * Physics.gravity.magnitude;

                string line = $"{frame}, {tilt:F4}, {force.magnitude:F4}, {mg:F4}, {torque.magnitude:F6}";
                _csv.AppendLine(line);
                Debug.Log("[TEST] " + line);
            }
        }

        // Final snapshot
        float finalTilt = Vector3.Angle(targetRb.transform.up, Vector3.up);
        Vector3 finalTorque = GetLastTorque(target);
        Debug.Log($"[TEST] A3 RESULT: settled tilt = {finalTilt:F3}° (expected 26.565°), |τ| = {finalTorque.magnitude:E2} N·m");
        Debug.Log("[TEST]\n" + _csv.ToString());
    }

    // ── A5: Wave zero regression ──────────────────────────────────────────
    // With all wave steepnesses = 0, buoyancy must be identical to flat water.
    // Run both with waves on (steepness 0) and compare against A1 results.
    IEnumerator RunA5()
    {
        Debug.Log("[TEST] A5: Ensure WaveManager steepnesses are 0 before running this.");
        Debug.Log("[TEST] A5: Results should be identical to A1. Compare the two CSVs.");

        // Re-run A1 logic — output will have waves active but at zero amplitude.
        yield return RunA1();
    }

    // ── A6: Adaptive vs Linear A/B ────────────────────────────────────────
    // Identical bodies float side by side — linear, adaptive (N=4), adaptive (N=8), and dense (ground truth).
    // Log mean height and mean tilt of each over time.
    IEnumerator RunA6()
    {
        if (adaptiveBody == null || adaptiveRb == null || denseBody == null || denseRb == null || adaptiveBody2 == null || adaptiveRb2 == null)
        {
            Debug.LogError("[TEST] A6: Assign AdaptiveBody, AdaptiveBody2, DenseBody, and their Rbs in the inspector.");
            yield break;
        }

        _csv.AppendLine("// A6 Adaptive vs Linear A/B");
        _csv.AppendLine("// frame, linear_height_m, adapt4_height_m, adapt8_height_m, dense_height_m, linear_tilt_deg, adapt4_tilt_deg, adapt8_tilt_deg, dense_tilt_deg");
        Debug.Log("[TEST] A6 header: frame, linear_height_m, adapt4_height_m, adapt8_height_m, dense_height_m, linear_tilt_deg, adapt4_tilt_deg, adapt8_tilt_deg, dense_tilt_deg");

        for (int frame = 0; frame < abDurationFrames; frame++)
        {
            yield return new WaitForFixedUpdate();

            if (frame % 10 == 0)
            {
                float linH   = targetRb.position.y;
                float adp4H  = adaptiveRb.position.y;
                float adp8H  = adaptiveRb2.position.y;
                float denseH = denseRb.position.y;
                
                float linT   = Vector3.Angle(targetRb.transform.up, Vector3.up);
                float adp4T  = Vector3.Angle(adaptiveRb.transform.up, Vector3.up);
                float adp8T  = Vector3.Angle(adaptiveRb2.transform.up, Vector3.up);
                float denseT = Vector3.Angle(denseRb.transform.up, Vector3.up);

                string line = $"{frame}, {linH:F4}, {adp4H:F4}, {adp8H:F4}, {denseH:F4}, {linT:F4}, {adp4T:F4}, {adp8T:F4}, {denseT:F4}";
                _csv.AppendLine(line);
                Debug.Log("[TEST] " + line);
            }
        }

        Debug.Log("[TEST]\n" + _csv.ToString());
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    // Uses reflection to read the private lastForce / lastTorque fields
    // from BuoyancyController without modifying that class.
    static Vector3 GetLastForce(BuoyancyController bc)
    {
        var f = typeof(BuoyancyController).GetField(
            "lastForce",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return f != null ? (Vector3)f.GetValue(bc) : Vector3.zero;
    }

    static Vector3 GetLastTorque(BuoyancyController bc)
    {
        var f = typeof(BuoyancyController).GetField(
            "lastTorque",
            System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
        return f != null ? (Vector3)f.GetValue(bc) : Vector3.zero;
    }

    static void FreezeBody(Rigidbody rb)
    {
        rb.isKinematic = true;
    }

    static void UnfreezeBody(Rigidbody rb)
    {
        rb.isKinematic = false;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }
}
