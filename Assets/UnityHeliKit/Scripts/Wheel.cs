using UnityEngine;
using System.Collections;

public class Wheel : MonoBehaviour {

    public float brakeTorque;

    WheelCollider wheelCollider;
	public Transform visualWheel;
	private Quaternion originalRotation;
    private Helicopter helicopter;

	void Start () {
		wheelCollider = GetComponent<WheelCollider>();
		originalRotation = visualWheel.transform.rotation;
        helicopter = GetComponentInParent<Helicopter>();
	}

	void Update () {

		if (visualWheel != null) {
			Vector3 position;
			Quaternion rotation;
			wheelCollider.GetWorldPose(out position, out rotation);

			visualWheel.transform.position = position;
			visualWheel.transform.rotation = rotation * originalRotation;
		}
        float brake = transform.localPosition.x < 0 ? helicopter.LeftBrake : helicopter.RightBrake;
        wheelCollider.brakeTorque = brake * brakeTorque;
	}
}
