using UnityEngine;

public class CameraController : MonoBehaviour
{
    [Header("Movement Settings")]
    [SerializeField] private float moveSpeed = 5f;
    [SerializeField] private float sprintMultiplier = 2f;
    [SerializeField] private float smoothTime = 0.1f;

    [Header("Look Settings")]
    [SerializeField] private float mouseSensitivity = 2f;
    [SerializeField] private bool invertY = false;
    [SerializeField] private float lookSmoothing = 0.1f;
    [SerializeField] private float minVerticalAngle = -80f;
    [SerializeField] private float maxVerticalAngle = 80f;

    // Private variables
    private Vector3 currentVelocity;
    private Vector3 targetPosition;
    private float rotationX = 0f;
    private float rotationY = 0f;
    private Vector2 currentMouseDelta;
    private Vector2 currentMouseDeltaVelocity;

    private void Start()
    {
        // Lock and hide cursor
        Cursor.lockState = CursorLockMode.Locked;
        Cursor.visible = false;

        // Initialize position
        targetPosition = transform.position;

        // Initialize rotation
        Vector3 rotation = transform.rotation.eulerAngles;
        rotationX = rotation.y;
        rotationY = rotation.x;
    }

    private void Update()
    {
        HandleMovementInput();
        HandleMouseLook();

        // Exit cursor lock with Escape key
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            ToggleCursorLock();
        }
    }

    private void HandleMovementInput()
    {
        // Get input axes
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        float upDown = 0f;

        // Up/Down movement with E and Q keys
        if (Input.GetKey(KeyCode.E))
            upDown = 1f;
        if (Input.GetKey(KeyCode.Q))
            upDown = -1f;

        // Calculate move direction in local space
        Vector3 moveDirection = new Vector3(horizontal, upDown, vertical);

        // Apply sprint multiplier if shift is held
        float currentMoveSpeed = Input.GetKey(KeyCode.LeftShift) ? moveSpeed * sprintMultiplier : moveSpeed;

        // Apply move speed and deltaTime
        moveDirection *= currentMoveSpeed * Time.deltaTime;

        // Transform direction to world space based on camera orientation
        moveDirection = transform.TransformDirection(moveDirection);

        // Update target position
        targetPosition += moveDirection;

        // Smooth movement
        transform.position = Vector3.SmoothDamp(transform.position, targetPosition, ref currentVelocity, smoothTime);
    }

    private void HandleMouseLook()
    {
        // Only process mouse look when cursor is locked
        if (Cursor.lockState == CursorLockMode.Locked)
        {
            // Get mouse input
            Vector2 targetMouseDelta = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));

            // Apply smooth mouse movement
            currentMouseDelta = Vector2.SmoothDamp(currentMouseDelta, targetMouseDelta, ref currentMouseDeltaVelocity, lookSmoothing);

            // Update rotation values
            rotationX += currentMouseDelta.x * mouseSensitivity;
            rotationY += currentMouseDelta.y * mouseSensitivity * (invertY ? 1 : -1);

            // Clamp vertical rotation
            rotationY = Mathf.Clamp(rotationY, minVerticalAngle, maxVerticalAngle);

            // Apply rotation
            transform.rotation = Quaternion.Euler(rotationY, rotationX, 0);
        }
    }

    private void ToggleCursorLock()
    {
        if (Cursor.lockState == CursorLockMode.Locked)
        {
            Cursor.lockState = CursorLockMode.None;
            Cursor.visible = true;
        }
        else
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
    }
}