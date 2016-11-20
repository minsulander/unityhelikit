using UnityEngine;
using System.Collections;

public class Compass : InstrumentBehaviour {

    public Transform roseTransform;

    new void Start() { base.Start(); }
    new void Update() { base.Update(); }

    public override void UpdateInstrument() {
        roseTransform.localRotation = Quaternion.Euler(0, 0, (float)aircraft.model.Heading * 180 / Mathf.PI);
    }

}
