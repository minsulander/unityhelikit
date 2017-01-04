using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class CameraController : MonoBehaviour {

    public enum CameraMode {
        FIRST,

        INTERIOR,
        CHASE,

        LAST // don't remove :)
    }

    public CameraMode mode = CameraMode.INTERIOR;
    public Camera interiorCamera { get; set; }
    public Camera exteriorCamera { get; set; }

    // Chase parameters
    public Transform target;
    public float azimuthSensitivity = 1;
    public float elevationSensitivity = 1;
    public float radiusSensitivity = 100;

    public Text modeText;

    private AudioListener interiorAudioListener;
    private AudioListener exteriorAudioListener;
    private GameObject[] interiorObjects;
    private GameObject[] exteriorObjects;

    private float radius, azimuth, elevation;

    // Use this for initialization
    void Start () {
        GameObject textObject = GameObject.Find("Camera Mode Text");
        if (textObject != null) modeText = textObject.GetComponent<Text>();
        if (interiorCamera == null && target != null) interiorCamera = target.GetComponentInChildren<Camera>();
        if (exteriorCamera == null) exteriorCamera = GetComponent<Camera>();

        if (interiorCamera != null) interiorAudioListener = interiorCamera.gameObject.GetComponent<AudioListener>();
        if (exteriorCamera != null) exteriorAudioListener = exteriorCamera.gameObject.GetComponent<AudioListener>();

        CartesianToSpherical(target.transform.InverseTransformDirection(exteriorCamera.transform.position - target.position), out radius, out azimuth, out elevation);
        exteriorCamera.transform.LookAt(target);

        Apply();
	}
	
	// Update is called once per frame
	void Update () {
        if (Input.GetButtonDown("Camera Mode")) {
            mode++;
            if (mode == CameraMode.LAST) mode = CameraMode.FIRST + 1;
            Apply();
        }

        if (mode == CameraMode.CHASE) {
            azimuth -= Input.GetAxis("Mouse X") * azimuthSensitivity * Time.deltaTime;
            elevation -= Input.GetAxis("Mouse Y") * elevationSensitivity * Time.deltaTime;
            radius -= Input.GetAxis("Mouse ScrollWheel") * radiusSensitivity * Time.deltaTime;

            Vector3 direction;
            SphericalToCartesian(radius, azimuth, elevation, out direction);

            exteriorCamera.transform.position = target.transform.position + direction;
            //exteriorCamera.transform.position = target.transform.position + target.transform.TransformDirection(direction);
        }
        exteriorCamera.transform.LookAt(target);
    }

    private void Apply() {

        if (interiorObjects == null) interiorObjects = GameObject.FindGameObjectsWithTag("Interior");
        if (exteriorObjects == null) exteriorObjects = GameObject.FindGameObjectsWithTag("Exterior");

        if (mode == CameraMode.INTERIOR) {
            interiorCamera.enabled = interiorAudioListener.enabled = true;
            foreach (var obj in interiorObjects) obj.SetActive(true);
            exteriorCamera.enabled = exteriorAudioListener.enabled = false;
            foreach (var obj in exteriorObjects) obj.SetActive(false);
        } else {
            exteriorCamera.enabled = exteriorAudioListener.enabled = true;
            foreach (var obj in exteriorObjects) obj.SetActive(true);
            interiorCamera.enabled = interiorAudioListener.enabled = false;
            foreach (var obj in interiorObjects) obj.SetActive(false);
        }
        if (modeText != null) modeText.text = mode.ToString();
    }

    public static void CartesianToSpherical(Vector3 cartCoords, out float outRadius, out float outPolar, out float outElevation) {
        if (cartCoords.x == 0)
            cartCoords.x = Mathf.Epsilon;
        outRadius = Mathf.Sqrt((cartCoords.x * cartCoords.x)
                        + (cartCoords.y * cartCoords.y)
                        + (cartCoords.z * cartCoords.z));
        outPolar = Mathf.Atan(cartCoords.z / cartCoords.x);
        if (cartCoords.x < 0)
            outPolar += Mathf.PI;
        outElevation = Mathf.Asin(cartCoords.y / outRadius);
    }

    public static void SphericalToCartesian(float radius, float polar, float elevation, out Vector3 outCart) {
        float a = radius * Mathf.Cos(elevation);
        outCart.x = a * Mathf.Cos(polar);
        outCart.y = radius * Mathf.Sin(elevation);
        outCart.z = a * Mathf.Sin(polar);
    }

}
