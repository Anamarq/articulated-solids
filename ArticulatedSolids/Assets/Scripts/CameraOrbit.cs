using UnityEngine;

public class CameraOrbit : MonoBehaviour
{
    [Header("Target to orbit around")]
    public Transform target;         
    public float distance = 10.0f; 
    public float minDistance = 2f;  
    public float maxDistance = 20f;  

    [Header("Orbit Speed")]
    public float xSpeed = 120.0f;    
    public float ySpeed = 120.0f;   
    public float zoomSpeed = 5.0f;   

    [Header("Limits")]
    public float yMinLimit = -20f;    
    public float yMaxLimit = 80f;     


    private float x = 0.0f;
    private float y = 0.0f;
    private float currentDistance;
    private float desiredDistance;
    private Vector3 velocity = Vector3.zero;

    void Start()
    {
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        currentDistance = distance;
        desiredDistance = distance;

        if (GetComponent<Rigidbody>())
            GetComponent<Rigidbody>().freezeRotation = true;
    }

    void LateUpdate()
    {
        if (!target) return;

        if (Input.GetMouseButton(0) || Input.GetMouseButton(1))
        {
            x += Input.GetAxis("Mouse X") * xSpeed * Time.deltaTime;
            y -= Input.GetAxis("Mouse Y") * ySpeed * Time.deltaTime;

            y = ClampAngle(y, yMinLimit, yMaxLimit);
        }

        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (scroll != 0.0f)
        {
            desiredDistance -= scroll * zoomSpeed;
            desiredDistance = Mathf.Clamp(desiredDistance, minDistance, maxDistance);
        }

        currentDistance = Mathf.Lerp(currentDistance, desiredDistance, Time.deltaTime * 5f);
        Quaternion rotation = Quaternion.Euler(y, x, 0);
        Vector3 desiredPosition = rotation * new Vector3(0, 0, -currentDistance) + target.position;

        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref velocity, 0);
        transform.rotation = rotation;
    }

    static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F) angle += 360F;
        if (angle > 360F) angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}