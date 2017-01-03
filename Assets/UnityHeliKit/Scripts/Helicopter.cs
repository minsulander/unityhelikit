using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using MathNet.Numerics.LinearAlgebra;

public abstract class Helicopter : MonoBehaviour
{

    public bool airStart = true;

    public Text debugText;

    public abstract HeliSharp.Helicopter model { get; }
    protected Rigidbody body;

    // "Relay" controls to dynamics model
    public float Throttle {
        get { return model is HeliSharp.SingleMainRotorHelicopter ? (float)((HeliSharp.SingleMainRotorHelicopter)model).Engine.throttle : 0; }
        set { if (model is HeliSharp.SingleMainRotorHelicopter) ((HeliSharp.SingleMainRotorHelicopter)model).Engine.throttle = value; }
    }
    public bool TrimControl {
        get { return model.FCS.trimControl; }
        set { model.FCS.trimControl = value; }
    }
    public float Collective {
        get { return (float)model.Collective; }
        set { model.Collective = value; }
    }
    public float LongCyclic {
        get { return (float)model.LongCyclic; }
        set { model.LongCyclic = value; }
    }
    public float LatCyclic {
        get { return (float)model.LatCyclic; }
        set { model.LatCyclic = value; }
    }
    public float Pedal {
        get { return (float)model.Pedal; }
        set { model.Pedal = value; }
    }
    public float LeftBrake { get; set; }
    public float RightBrake { get; set; }

    protected virtual void FindComponents() {
        body = GetComponent<Rigidbody>();
        if (body == null) {
            body = gameObject.AddComponent<Rigidbody>();
            body.mass = (float)model.Mass;
            body.drag = body.angularDrag = 0f;
        }
        Transform centerOfMassTransform = transform.FindChild("CenterOfMass");
        if (centerOfMassTransform != null) body.centerOfMass = centerOfMassTransform.localPosition;
    }

    public virtual void Trim(bool initial)
    {
        Debug.Log("Trim" + (initial ? " initial" : ""));
        model.Rotation = body.rotation.FromUnity();
        if (initial) {
            model.RollAngle = 0;
            model.PitchAngle = 0;
            model.AngularVelocity = Vector<double>.Build.Dense(3);
            model.TrimInit();
        }

        // Trim for equilibrium
        try {
            model.Trim();
        } catch (HeliSharp.TrimmerException e) {
            Debug.LogException(e);
            enabled = false;
            if (debugText != null) debugText.text = "TRIM FAIL";
            return;
        }

        if (!initial || airStart) body.rotation = model.Rotation.ToUnity ();
    }

    public abstract void ToggleEngine();

    public float? GetHeight() {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, new Vector3(0,-1,0), out hit, 1000f)) {
            return transform.position.y - hit.point.y;
        }
        return null;
    }

}
