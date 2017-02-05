using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;

public class SingleMainRotorHelicopter : Helicopter {

	public Rotor mainRotor;
	public Rotor tailRotor;
	public Stabilizer horizontalStabilizer;
	public Stabilizer verticalStabilizer;
	public Fuselage fuselage;
	public FlightControlSystem fcs;
    public Engine engine;
    public GearBox gearBox;

	public override HeliSharp.Helicopter model { get { return _model; } }
    public HeliSharp.SingleMainRotorHelicopter _model { get; set; }

	private Transform mainRotorTransform;
	private Transform tailRotorTransform;
	private Transform horizontalStabilizerTransform;
	private Transform verticalStabilizerTransform;
	private Transform fuselageTransform;

    private float mainRotorSpinAngle, tailRotorSpinAngle;

	public SingleMainRotorHelicopter() {
		_model = new HeliSharp.SingleMainRotorHelicopter ();
	}

	void Start () {
		FindComponents();
		ParametrizeModelsFromUnity();
		Trim(true);
	}

	public override void Trim(bool initial)
	{
	    base.Trim(initial);
		if (initial) _model.InitEngine(airStart);
	}

    public override void ToggleEngine() {
        if (_model.Engine.phase == Engine.Phase.CUTOFF) {
            _model.Engine.phase = Engine.Phase.START;
        } else if (_model.Engine.phase == Engine.Phase.RUN) {
            _model.Engine.phase = Engine.Phase.CUTOFF;
        }
    }

    public virtual void FixedUpdate () {
		if (body == null) return;

        base.FixedUpdate();

        if (debugText != null) {
			string text = "";
            text += _model.Engine.phase + " THR " + _model.Engine.throttle + " RPM E " + (_model.Engine.rotspeed * 9.5493).ToStr() + " M/R " + (_model.MainRotor.RotSpeed * 9.5493).ToStr() + "\n";
			text += "WASH " + Mathf.Round((float)_model.MainRotor.WashVelocity.Norm(2)) + "\n";
			debugText.text += text;
		}

        // Move rotors
        if (mainRotorTransform != null) {
			mainRotorSpinAngle += -mainRotor.rotdir * (float)mainRotor.RotSpeed * 180f / Mathf.PI * Time.deltaTime;
            while (mainRotorSpinAngle > 360f) mainRotorSpinAngle -= 360f;
            mainRotorTransform.localRotation = Quaternion.Euler(new Vector3((float)mainRotor.beta_cos * 180f / Mathf.PI, 0, (float)mainRotor.beta_sin * 180f / Mathf.PI)) * _model.MainRotor.Rotation.ToUnity()
                * Quaternion.AngleAxis(mainRotorSpinAngle, new Vector3(0, 1, 0));
        }
        if (tailRotorTransform != null) {
			tailRotorSpinAngle += -tailRotor.rotdir * (float)tailRotor.RotSpeed * 180f / Mathf.PI * Time.deltaTime;
            while (tailRotorSpinAngle > 360f) tailRotorSpinAngle -= 360f;
            tailRotorTransform.localRotation = _model.TailRotor.Rotation.ToUnity() * Quaternion.AngleAxis(tailRotorSpinAngle, new Vector3(0, 1, 0));
        }

    }

    public void OnDrawGizmosSelected() {
		if (mainRotor == null || model == null) return;

        // Draw debug visualization
        float visualizationScale = (float)(mainRotor.R / model.Mass / 9.81);
        if (mainRotorTransform != null) {
            Debug.DrawLine(transform.TransformPoint(mainRotor.Translation.ToUnity()), transform.TransformPoint(mainRotor.Translation.ToUnity() + (mainRotor.Rotation * mainRotor.Force).ToUnity() * visualizationScale), Color.yellow);
            DebugDrawRotor(mainRotorTransform, mainRotor, 32);
            // Downwash
            Debug.DrawLine(transform.position, transform.TransformPoint(-mainRotor.WashVelocity.ToUnity() / 3), Color.blue);
        }
        if (tailRotorTransform != null) {
            Debug.DrawLine(transform.TransformPoint(tailRotor.Translation.ToUnity()), transform.TransformPoint(tailRotor.Translation.ToUnity() + (tailRotor.Rotation * tailRotor.Force).ToUnity() * visualizationScale * 3), Color.yellow);
            DebugDrawRotor(tailRotorTransform, tailRotor, 16);
        }
        if (horizontalStabilizerTransform != null) {
            Debug.DrawLine(horizontalStabilizerTransform.position, horizontalStabilizerTransform.TransformPoint(horizontalStabilizer.Force.ToUnity() * visualizationScale * 100), Color.yellow);
            DebugDrawStabilizer(horizontalStabilizerTransform, horizontalStabilizer);
            // Downwash
            Debug.DrawLine(horizontalStabilizerTransform.position, horizontalStabilizerTransform.TransformPoint(-horizontalStabilizer.Velocity.ToUnity() / 3), Color.cyan);
        }
        if (verticalStabilizerTransform != null) {
            Debug.DrawLine(verticalStabilizerTransform.position, verticalStabilizerTransform.TransformPoint(verticalStabilizer.Force.ToUnity() * visualizationScale * 100), Color.yellow);
            DebugDrawStabilizer(verticalStabilizerTransform, verticalStabilizer);
            // Downwash
            Debug.DrawLine(verticalStabilizerTransform.position, verticalStabilizerTransform.TransformPoint(-verticalStabilizer.Velocity.ToUnity() / 3), Color.cyan);
        }
        if (fuselageTransform != null) {
            Debug.DrawLine(fuselageTransform.position, fuselageTransform.TransformPoint(fuselage.Force.ToUnity() * visualizationScale * 100), Color.yellow);
            // Downwash
            Debug.DrawLine(fuselageTransform.position, fuselageTransform.TransformPoint(-fuselage.Velocity.ToUnity() / 3), Color.cyan);
        }
    }

	public override void ParametrizeUnityFromModel() {
		// Set mass and inertia
		body.mass = (float)model.Mass;
		Vector3 inertia = (model.Inertia * Vector<double>.Build.DenseOfArray(new double[] { 1, 1, 1 })).ToUnity();
		inertia.y *= -1; // all inertia tensor components must be positive
		body.inertiaTensor = inertia;
		Debug.Log("Inertia tensor " + body.inertiaTensor);
		// TODO what about body.inertiaTensorRotation ?

		// Sub models
		mainRotor = _model.MainRotor;
		mainRotorTransform = transform.FindChild("MainRotor");
		if (mainRotorTransform == null) {
			GameObject mainRotorGO = new GameObject("MainRotor");
			mainRotorTransform = mainRotorGO.transform;
			mainRotorTransform.parent = transform;
		}
		mainRotorTransform.localPosition = _model.MainRotor.Translation.ToUnity();
		mainRotorTransform.localRotation = _model.MainRotor.Rotation.ToUnity();

		tailRotor = _model.TailRotor;
		tailRotorTransform = transform.FindChild("TailRotor");
		if (tailRotorTransform == null) {
			GameObject tailRotorGO = new GameObject("TailRotor");
			tailRotorTransform = tailRotorGO.transform;
			tailRotorTransform.parent = transform;
		}
		tailRotorTransform.localPosition = _model.TailRotor.Translation.ToUnity();
		tailRotorTransform.localRotation = _model.TailRotor.Rotation.ToUnity();

		horizontalStabilizer = _model.HorizontalStabilizer;
		horizontalStabilizerTransform = transform.FindChild("HorizontalStabilizer");
		if (horizontalStabilizerTransform == null) {
			GameObject horizontalStabilizerGO = new GameObject("HorizontalStabilizer");
			horizontalStabilizerTransform = horizontalStabilizerGO.transform;
			horizontalStabilizerTransform.parent = transform;
		}
		horizontalStabilizerTransform.localPosition = _model.HorizontalStabilizer.Translation.ToUnity();
		horizontalStabilizerTransform.localRotation = _model.HorizontalStabilizer.Rotation.ToUnity();

		verticalStabilizer = _model.VerticalStabilizer;
		verticalStabilizerTransform = transform.FindChild("VerticalStabilizer");
		if (verticalStabilizerTransform == null) {
			GameObject verticalStabilizerGO = new GameObject("VerticalStabilizer");
			verticalStabilizerTransform = verticalStabilizerGO.transform;
			verticalStabilizerTransform.parent = transform;
		}
		verticalStabilizerTransform.localPosition = _model.VerticalStabilizer.Translation.ToUnity();
		verticalStabilizerTransform.localRotation = _model.VerticalStabilizer.Rotation.ToUnity();

		fuselage = _model.Fuselage;
		fuselageTransform = transform.FindChild("Fuselage");
		if (fuselageTransform == null) {
			GameObject fuselageGO = new GameObject("Fuselage");
			fuselageTransform = fuselageGO.transform;
			fuselageTransform.parent = transform;
		}
		fuselageTransform.localPosition = _model.Fuselage.Translation.ToUnity();
		fuselageTransform.localRotation = _model.Fuselage.Rotation.ToUnity();

		fcs = model.FCS;
        engine = _model.Engine;
        gearBox = _model.GearBox;
	}

	public override void ParametrizeModelsFromUnity() {
		_model.LoadDefault ();
		// Set mass and inertia
		//model.Mass = body.mass;
		//model.Inertia = Matrix<double>.Build.DenseOfDiagonalVector(body.inertiaTensor.FromUnity());
		Debug.Log ("Mass " + model.Mass.ToStr ());
		Debug.Log ("Inertia " + model.Inertia.ToStr ());
		// TODO what about body.inertiaTensorRotation ?

		// Add sub models
		mainRotorTransform = transform.FindChild("MainRotor");
		if (mainRotorTransform == null) {
			Debug.LogError("No MainRotor child");
		} else {
			_model.MainRotor = mainRotor;
			_model.MainRotor.Translation = mainRotorTransform.localPosition.FromUnity();
			_model.MainRotor.Rotation = mainRotorTransform.localRotation.FromUnity();
		}
		tailRotorTransform = transform.FindChild("TailRotor");
		if (tailRotorTransform == null) {
			Debug.LogError("No TailRotor child");
		} else {
			_model.TailRotor = tailRotor;
			_model.TailRotor.Translation = tailRotorTransform.localPosition.FromUnity();
			_model.TailRotor.Rotation = tailRotorTransform.localRotation.FromUnity();
		}
		horizontalStabilizerTransform = transform.FindChild("HorizontalStabilizer");
		if (horizontalStabilizerTransform != null) {
			_model.HorizontalStabilizer = horizontalStabilizer;
			_model.HorizontalStabilizer.Translation = horizontalStabilizerTransform.localPosition.FromUnity();
			_model.HorizontalStabilizer.Rotation = horizontalStabilizerTransform.localRotation.FromUnity();
		}
		verticalStabilizerTransform = transform.FindChild("VerticalStabilizer");
		if (verticalStabilizerTransform != null) {
			_model.VerticalStabilizer = verticalStabilizer;
			_model.VerticalStabilizer.Translation = verticalStabilizerTransform.localPosition.FromUnity();
			_model.VerticalStabilizer.Rotation = verticalStabilizerTransform.localRotation.FromUnity();
		}
		fuselageTransform = transform.FindChild("Fuselage");
		if (fuselageTransform != null) {
			_model.Fuselage = fuselage;
			_model.Fuselage.Translation = fuselageTransform.localPosition.FromUnity();
			_model.Fuselage.Rotation = fuselageTransform.localRotation.FromUnity();
		}

		model.FCS = fcs;
        _model.Engine = engine;
        _model.GearBox = gearBox;
		model.Gravity.Enabled = false;
	}
}
