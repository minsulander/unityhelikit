using UnityEngine;
using System.Collections;

public class VerticalSpeedIndicator : InstrumentBehaviour {

    public Transform needle;

    public float zeroAngle = 90;
    public float multiplier = -11.81102364f; // 196.850394 * 180 / 3000

    new void Start() { base.Start(); }
    new void Update() { base.Update(); }

    public override void UpdateInstrument() {
        needle.localRotation = Quaternion.Euler(0, 0, zeroAngle + multiplier * aircraft.GetComponent<Rigidbody>().velocity.y);
    }

}
