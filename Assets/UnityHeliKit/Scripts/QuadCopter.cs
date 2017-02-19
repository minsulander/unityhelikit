using HeliSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QuadCopter : Helicopter {

    public override HeliSharp.Helicopter model { get { return _model; } }
    public HeliSharp.QuadCopter _model { get; set; }

    public Rotor rotors;

    public QuadCopter() {
        _model = new HeliSharp.QuadCopter();
    }

    public override void FixedUpdate() {
        if (body == null) return;

        base.FixedUpdate();
        /*
        if (debugText != null) {
            string text = "";
            text += "CONE " + _model.Rotors[0].beta_0;
            debugText.text += text;
        }
        */
    }

    public override void ParametrizeUnityFromModel() {
        base.ParametrizeUnityFromModel();
        rotors = new Rotor();
        rotors.CopyParameters(_model.Rotors[0]);
    }

    public override void ParametrizeModelsFromUnity() {
        model.LoadDefault();
        for (var i = 0; i < _model.Rotors.Length; i++) {
            _model.Rotors[i].CopyParameters(rotors);
            if (i % 2 == 1) _model.Rotors[i].rotdir = -1;
        }
        base.ParametrizeModelsFromUnity();
    }
}
