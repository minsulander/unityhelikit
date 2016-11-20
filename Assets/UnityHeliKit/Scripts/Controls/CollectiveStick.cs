using UnityEngine;
using System.Collections;

public class CollectiveStick : MonoBehaviour {

	public float deflection = 25f;

    private Helicopter helicopter;
    private Quaternion originalRotation;

    void Start () {
		helicopter = GetComponentInParent<Helicopter> ();
        originalRotation = transform.localRotation;
    }

    void Update () {
		transform.localRotation = originalRotation * Quaternion.Euler (new Vector3 (((float)helicopter.fcs.CollectiveCommand/2f+0.5f) * deflection, 0, 0));
	}
}
