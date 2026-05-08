using UnityEngine;
using UnityEngine.UIElements;

public class BuoyancyController : MonoBehaviour
{
    // For visuals
    [Header("Debug")]
    [SerializeField] bool showDebug = true;
    [SerializeField] float normalLength = 0.15f;
    [SerializeField] float forceScale = 0.01f;

    [Header("Enviorment")]
    [SerializeField] float rho = 1000f;
    [SerializeField] float gravity = 9.81f;
    [Header("Damping")]
    [SerializeField, Range(0.9f, 1.0f)] float angularAlpha = 0.98f;
    [SerializeField] float linearDrag = 3f;  // drag force when in water (0 = no drag)
    [Header("Validation")]
    [SerializeField] bool logEquilibrium = false;

    private Rigidbody rb;
    private int handle;
    float[] res = new float[6];
    [SerializeField] Vector3 lastForce;
    [SerializeField] Vector3 lastTorque;
    Vector3 L;  // angular momentum

    void Start()
    {
        // Get all data needed for calculations
        rb = GetComponent<Rigidbody>();

        // Init angular momentum from current angular velocity
        Quaternion q = rb.inertiaTensorRotation;
        Vector3 w = rb.angularVelocity;
        L = q * Vector3.Scale(rb.inertiaTensor, Quaternion.Inverse(q) * w);

        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        Vector3[] verts = mesh.vertices;
        int[] tris = mesh.triangles;
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

        float[] matrix = new float[16];
        Matrix4x4 m = transform.localToWorldMatrix;
        for (int i = 0; i < 16; i++)
        {
            matrix[i] = m[i];
        }
        NativeBridge.ComputeBuoyancy(handle, matrix, null, 0, res);
        lastForce = new Vector3(res[0], res[1], res[2]);
        lastTorque = new Vector3(res[3], res[4], res[5]);

        float dt = Time.fixedDeltaTime;

        // Linear dampning, unity handles gravity. We apply buoyancy + water drag.
        rb.AddForce(lastForce, ForceMode.Force);
        if (lastForce.sqrMagnitude > 0.0001f)  // in water
            rb.AddForce(-linearDrag * rb.linearVelocity, ForceMode.Force);

        //lastTorque = Vector3.Dot(lastTorque, Vector3.right) * Vector3.right;
        // Angular dampning, integrate torque -> momentum -> damp -> velocity
        L += lastTorque * dt;
        L *= angularAlpha;
        Quaternion q = rb.inertiaTensorRotation;
        Vector3 L_local = Quaternion.Inverse(q) * L;
        Vector3 omega_local;
        omega_local.x = L_local.x / rb.inertiaTensor.x;
        omega_local.y = L_local.y / rb.inertiaTensor.y;
        omega_local.z = L_local.z / rb.inertiaTensor.z;
        rb.angularVelocity = q * omega_local;
    }

    private int frameCount;
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
    }

    // For debug visuals
    void OnDrawGizmos()
    {
        if (!showDebug) return;

        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        if (mesh == null) return;

        Vector3[] verts = mesh.vertices;
        int[] tris = mesh.triangles;

        for (int i = 0; i < tris.Length; i += 3)
        {
            Vector3 a = transform.TransformPoint(verts[tris[i]]);
            Vector3 b = transform.TransformPoint(verts[tris[i + 1]]);
            Vector3 c = transform.TransformPoint(verts[tris[i + 2]]);

            // Classify triangle against water plane y=0
            bool aBelow = a.y < 0, bBelow = b.y < 0, cBelow = c.y < 0;
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

            // Normal
            Vector3 centroid = (a + b + c) / 3f;
            Vector3 normal = Vector3.Cross(b - a, c - a).normalized;
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(centroid, normal * normalLength);
        }
    }


}
