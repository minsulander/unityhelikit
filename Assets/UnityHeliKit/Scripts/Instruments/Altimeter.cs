using UnityEngine;
using System.Collections;

public class Altimeter : InstrumentBehaviour {

    public Transform needle;

    public float zeroAngle = 0;
    public float multiplier = -1.181102364f; // 3.2808399 * 360 / 1000

    new void Start() { base.Start(); }
    new void Update() { base.Update(); }

    public override void UpdateInstrument() {
        needle.localRotation = Quaternion.Euler(0, 0, zeroAngle + multiplier * aircraft.transform.position.y );
    }

}
