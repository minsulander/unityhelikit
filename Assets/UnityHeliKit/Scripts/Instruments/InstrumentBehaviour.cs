﻿using UnityEngine;
using System.Collections;

public abstract class InstrumentBehaviour : MonoBehaviour {

    public Helicopter aircraft;

    public virtual void Start () {
        if (aircraft == null) aircraft = GetComponentInParent<Helicopter>();
    }

    public virtual void Update () {
        if (aircraft != null && aircraft.model != null) UpdateInstrument();
	}

    public abstract void UpdateInstrument();
}
