using UnityEngine;
using System.Collections;

public class Pedal : MonoBehaviour {

	public float deflection = 50f;
	public bool inverted;

    private Helicopter helicopter;
    private Quaternion originalRotation;

    void Start () {
		helicopter = GetComponentInParent<Helicopter> ();
        originalRotation = transform.localRotation;
    }

    void Update () {
		transform.localRotation = originalRotation * Quaternion.Euler (new Vector3 ((float)helicopter.fcs.PedalCommand * deflection * (inverted ? -1f : 1f), 0, 0));
	}
}
