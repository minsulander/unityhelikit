using UnityEngine;
using System.Collections;

public class AirspeedIndicator : InstrumentBehaviour {

    public Transform needle;

    public float zeroAngle;
    public float multiplier = -4.08198f;

    new void Start() { base.Start(); }
    new void Update() { base.Update(); }

    public override void UpdateInstrument() {
        needle.localRotation = Quaternion.Euler(0, 0, zeroAngle + multiplier * (float)aircraft.model.Velocity[0]);
    }

}
