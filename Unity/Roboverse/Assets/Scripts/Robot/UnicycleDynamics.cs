using UnityEngine;

public class UnicycleDynamics : MonoBehaviour
{
    [Header("Dynamics parameters")]
    public float z = 0.0f;
    public float x = 0.0f;
    public float theta = 0.0f;

    [Header("Linear and angular speeds")]
    public float linearSpeed = 0.0f;     // m/s
    public float angularSpeed = 0.0f;    // rad/s

    void Start()
    {
        // Initialize position/orientation
        x = transform.position.x;
        z = transform.position.z;
        theta = transform.eulerAngles.y * Mathf.Deg2Rad;
    }

    void Update()
    {
        // Integrate dynamics
        float dt = Time.deltaTime;

        // Update the unicycle model
        z += linearSpeed * Mathf.Cos(theta) * dt;
        x += linearSpeed * Mathf.Sin(theta) * dt;
        theta += angularSpeed * dt;

        // Update GameObject transform on the environment
        transform.position = new Vector3(x, transform.position.y, z);
        transform.eulerAngles = new Vector3(transform.eulerAngles.x, theta * Mathf.Rad2Deg, transform.eulerAngles.z);
    }
}
