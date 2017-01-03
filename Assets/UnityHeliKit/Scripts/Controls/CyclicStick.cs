using UnityEngine;
using System.Collections;

public class CyclicStick : MonoBehaviour {

	public float longitudinalDeflection = 15f;
	public float lateralDeflection = 15f;

    private SingleMainRotorHelicopter helicopter;
    private Quaternion originalRotation;

	void Start () {
		helicopter = GetComponentInParent<SingleMainRotorHelicopter> ();
        originalRotation = transform.localRotation;
	}
	
	void Update () {
		transform.localRotation = originalRotation * Quaternion.Euler (new Vector3 ((float)-helicopter.fcs.LongCommand * longitudinalDeflection, (float)-helicopter.fcs.LatCommand * lateralDeflection, 0));
	}
}
