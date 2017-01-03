using UnityEngine;
using System.Collections;

public class CollectiveStick : MonoBehaviour {

	public float deflection = 25f;

    private SingleMainRotorHelicopter helicopter;
    private Quaternion originalRotation;

    void Start () {
		helicopter = GetComponentInParent<SingleMainRotorHelicopter> ();
        originalRotation = transform.localRotation;
    }

    void Update () {
		transform.localRotation = originalRotation * Quaternion.Euler (new Vector3 (((float)helicopter.fcs.CollectiveCommand/2f+0.5f) * deflection, 0, 0));
	}
}
