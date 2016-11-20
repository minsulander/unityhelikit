using UnityEngine;
using System.Collections;

public class AttitudeIndicator : InstrumentBehaviour {

    public Transform rollTransform;
    public Transform pitchTransform;
    public float rollMultiplier = 1;
    public float pitchMultiplier = -2;

    new void Start() { base.Start(); }
    new void Update() { base.Update(); }

	public override void UpdateInstrument () {
        pitchTransform.localPosition = new Vector3(0, pitchMultiplier * (float)aircraft.model.PitchAngle * 180 / Mathf.PI, 0);
        rollTransform.localRotation = Quaternion.Euler(0, 0, rollMultiplier * (float)aircraft.model.RollAngle * 180 / Mathf.PI);
	}
}
