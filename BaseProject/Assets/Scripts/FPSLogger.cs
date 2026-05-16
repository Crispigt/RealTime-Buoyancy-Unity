using UnityEngine;

/// <summary>
/// Simple script to log average FPS to the console every second.
/// </summary>
public class FPSLogger : MonoBehaviour
{
    private int frameCount = 0;
    private float timePassed = 0f;
    private float updateInterval = 1f; // Log every 1 second

    void Update()
    {
        frameCount++;
        timePassed += Time.unscaledDeltaTime;

        if (timePassed >= updateInterval)
        {
            float fps = frameCount / timePassed;
            Debug.Log($"[STRESS TEST] Average FPS: {fps:F0}");
            
            // Reset for the next second
            frameCount = 0;
            timePassed = 0f;
        }
    }
}
