using UnityEngine;
using System.Collections;

public class Downwash : MonoBehaviour {

	private Helicopter helicopter;
	private WindZone windZone;

	public float minWashVelocity = 5f;
	public float maxWashVelocity = 10f;

	private float origMain, origTurbulence, origPulseMagnitude;

	void Start () {
		helicopter = GetComponentInParent<Helicopter> ();
		windZone = GetComponent<WindZone> ();
		origMain = windZone.windMain;
		origTurbulence = windZone.windTurbulence;
		origPulseMagnitude = windZone.windPulseMagnitude;
	}

	void Update () {
        float velocity = (float) helicopter.mainRotor.WashVelocity.Norm(2);
        float scale = Mathf.Clamp01 ((velocity - minWashVelocity) / (maxWashVelocity - minWashVelocity));
		windZone.windMain = scale * origMain;
		windZone.windTurbulence = scale * origTurbulence;
		windZone.windPulseMagnitude = scale * origPulseMagnitude;
	}
}
