using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 10f; // Camera movement speed
    public float zoomSpeed = 5f; // Camera zoom speed
    public float rotationSpeed = 100f; // Camera rotation speed
    public float sprintMultiplier = 2f; // Speed multiplier when sprinting

    public float minZoom = 5f; // Minimum zoom distance
    public float maxZoom = 50f; // Maximum zoom distance

    private Transform cameraTransform; // To keep rotation separate

    void Start()
    {
        cameraTransform = Camera.main.transform; // Assign the main camera's transform

        // Check if operating system is macOS
        if (SystemInfo.operatingSystemFamily == OperatingSystemFamily.MacOSX) {
            rotationSpeed *= 30f;
        }
    }

    void Update()
    {
        HandleMovement();
        HandleZoom();
        HandleMouseForRotation();
    }

    void HandleMovement()
    {
        float horizontal = Input.GetAxis("Horizontal"); // Input from A/D or left/right arrow keys
        float vertical = Input.GetAxis("Vertical"); // Input from W/S or up/down arrow keys
        float speedModifier = Input.GetKey(KeyCode.LeftShift) ? sprintMultiplier : 1f; // Adjust speed when sprinting

        Vector3 direction = cameraTransform.right * horizontal + cameraTransform.forward * vertical;
        transform.position += direction * moveSpeed * speedModifier * Time.deltaTime;
    }

    void HandleZoom()
    {
        float scroll = Input.GetAxis("Mouse ScrollWheel"); // Input from mouse scroll wheel
        Vector3 zoomDirection = cameraTransform.forward * scroll * zoomSpeed;

        float distance = Vector3.Distance(transform.position + zoomDirection, Vector3.zero);
        if (distance > minZoom && distance < maxZoom)
        {
            transform.position += zoomDirection;
        }
    }

    void HandleRotation()
    {
        float mouseX = Input.GetAxis("Mouse X");
        float mouseY = Input.GetAxis("Mouse Y");

        // Horizontal rotation
        transform.Rotate(Vector3.up, mouseX * rotationSpeed * Time.deltaTime, Space.World);

        // Vertical rotation
        cameraTransform.Rotate(Vector3.right, -mouseY * rotationSpeed * Time.deltaTime, Space.Self);
    }

    void HandleMouseForRotation() {
        // If right mouse button is pressed (on all operating systems)
        if (Input.GetMouseButton(1))
        {
            HandleRotation();
        }

        // If option key and the left mouse button are pressed on macOS
        if (SystemInfo.operatingSystemFamily == OperatingSystemFamily.MacOSX && Input.GetKey(KeyCode.LeftAlt) && Input.GetMouseButton(0))
        {
            HandleRotation();
        }
    }
}
