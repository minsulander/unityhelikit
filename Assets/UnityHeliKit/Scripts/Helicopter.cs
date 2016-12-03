using UnityEngine;
using UnityEngine.UI;
using System.Collections;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.Generic;

public class Helicopter : MonoBehaviour {

	public bool airStart = true;

	public Rotor mainRotor;
	public Rotor tailRotor;
	public Stabilizer horizontalStabilizer;
	public Stabilizer verticalStabilizer;
	public Fuselage fuselage;
	public FlightControlSystem fcs;
    public Engine engine;
    public GearBox gearBox;

	public SingleMainRotorHelicopter model { get; private set; }
	private Rigidbody body;

	public Text debugText;

    // "Relay" controls to dynamics model
    public float Throttle {
        get { return (float)model.Engine.throttle; }
        set { model.Engine.throttle = value; }
    }
    public bool TrimControl {
        get { return fcs.trimControl; }
        set { fcs.trimControl = value; }
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


	private Transform mainRotorTransform;
	private Transform tailRotorTransform;
	private Transform horizontalStabilizerTransform;
	private Transform verticalStabilizerTransform;
	private Transform fuselageTransform;

    private float mainRotorSpinAngle, tailRotorSpinAngle;

	public Helicopter() {
		model = new SingleMainRotorHelicopter ();
	}


	void Start () {
		FindComponents();
		ParametrizeModelsFromUnity();
		Trim(true);
	}

	public void Trim(bool initial) {
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
        } catch (TrimmerException e) {
            Debug.LogException(e);
			enabled = false;
			if (debugText != null) debugText.text = "TRIM FAIL";
			return;
        }

		if (!initial || airStart) body.rotation = model.Rotation.ToUnity ();
		if (initial) model.InitEngine(airStart);
	}

    public void ToggleEngine() {
        if (model.Engine.phase == Engine.Phase.CUTOFF) {
            model.Engine.phase = Engine.Phase.START;
        } else if (model.Engine.phase == Engine.Phase.RUN) {
            model.Engine.phase = Engine.Phase.CUTOFF;
        }
    }

    void FixedUpdate () {
		if (body == null) return;


		// Set velocities and attitues from rigid body simulation
		model.Velocity = transform.InverseTransformDirection(body.velocity).FromUnity();
		model.AngularVelocity = -transform.InverseTransformDirection(body.angularVelocity).FromUnity();
		model.Translation = transform.position.FromUnity();
		model.Rotation = transform.rotation.FromUnity();

		// Set height from ray trace (for ground effect)
		model.Height = GetHeight() ?? 999;

        if (model.Collective < -1) model.Collective = -1;

        if (debugText != null) {
			string text = "";
			text += "COLL " + model.Collective.ToStr() + " LONG " + model.LongCyclic.ToStr() + " LAT " + model.LatCyclic.ToStr() + " PED " + model.Pedal.ToStr() + "\n";
			text += "ATT x " + Mathf.Round((float)model.Attitude.x() * 180f / Mathf.PI) + " y " + Mathf.Round((float)model.Attitude.y() * 180f / Mathf.PI) + " z " + Mathf.Round((float)model.Attitude.z() * 180f / Mathf.PI) + "\n";
			//text += "PITCH " + (model.PitchAngle * 180.0 / Mathf.PI).ToStr() + " ROLL " + (model.RollAngle * 180.0 / Mathf.PI).ToStr() + "\n";
			text += "ALT " + Mathf.Round(transform.position.y) + "m HEIGHT " + Mathf.Round((float)model.Height) + "m\n";
			text += "SPEED " + Mathf.Round((float)model.Velocity.x() * 1.9438f) + "kts LAT " + Mathf.Round((float)model.Velocity.y() * 1.9438f) + " kts VERT " + Mathf.Round(body.velocity.y * 197f) + " fpm\n";
            //text += "VEL " + (int)model.Velocity.x() + " " + (int)model.Velocity.y() + " " + (int)model.Velocity.z() + "\n";
            text += "AVEL " + (int)(model.AngularVelocity.x()*100) + " " + (int)(model.AngularVelocity.y()*100) + " " + (int)(model.AngularVelocity.z()*100) + "\n";
            text += model.Engine.phase + " THR " + model.Engine.throttle + " RPM E " + (model.Engine.rotspeed * 9.5493).ToStr() + " M/R " + (model.MainRotor.RotSpeed * 9.5493).ToStr() + "\n";
			//text += "F " + (int)model.Force.x() + " " + (int)model.Force.y() + " " + (int)model.Force.z() + "\n";
			//text += "M " + (int)model.Torque.x() + " " + (int)model.Torque.y() + " " + (int)model.Torque.z() + "\n";
			//text += "M/R F " + (int)mainRotor.Force.x() + " " + (int)mainRotor.Force.y() + " " + (int)mainRotor.Force.z() + "\n";
			//text += "M/R M " + (int)mainRotor.Torque.x() + " " + (int)mainRotor.Torque.y() + " " + (int)mainRotor.Torque.z() + "\n";
			//text += "H/S Fz " + (int)horizontalStabilizer.Force.z () + " V/S Fz " + (int)verticalStabilizer.Force.z () + "\n";
			//text += "FUSE Mz " + (int)fuselage.Torque.z () + "\n";
			//text += "uF " + (int)force.x + " " + (int)force.y + " " + (int)force.z + "\n";
			//text += "uM " + (int)torque.x + " " + (int)torque.y + " " + (int)torque.z + "\n";
			text += "WASH " + Mathf.Round((float)model.MainRotor.WashVelocity.Norm(2)) + "\n";
            if (LeftBrake > 0.01f || RightBrake > 0.01f) text += "BRAKE\n";
			debugText.text = text;
		}

		// Update dynamics
		model.Update(Time.fixedDeltaTime);

        // Set force and torque/moment in rigid body simulation
        Vector3 force = model.Force.ToUnity();
        Vector3 torque = -model.Torque.ToUnity(); // minus because Unity uses a left-hand coordinate system
        body.AddRelativeForce(force);
		body.AddRelativeTorque(torque);

        // Move rotors
        if (mainRotorTransform != null) {
			mainRotorSpinAngle += -mainRotor.rotdir * (float)mainRotor.RotSpeed * 180f / Mathf.PI * Time.deltaTime;
            while (mainRotorSpinAngle > 360f) mainRotorSpinAngle -= 360f;
            mainRotorTransform.localRotation = Quaternion.Euler(new Vector3((float)mainRotor.beta_cos * 180f / Mathf.PI, 0, (float)mainRotor.beta_sin * 180f / Mathf.PI)) * model.MainRotor.Rotation.ToUnity()
                * Quaternion.AngleAxis(mainRotorSpinAngle, new Vector3(0, 1, 0));
        }
        if (tailRotorTransform != null) {
			tailRotorSpinAngle += -tailRotor.rotdir * (float)tailRotor.RotSpeed * 180f / Mathf.PI * Time.deltaTime;
            while (tailRotorSpinAngle > 360f) tailRotorSpinAngle -= 360f;
            tailRotorTransform.localRotation = model.TailRotor.Rotation.ToUnity() * Quaternion.AngleAxis(tailRotorSpinAngle, new Vector3(0, 1, 0));
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

    private void DebugDrawRotor(Transform transform, Rotor rotor, int numSegments) {
		for (int i = 0; i < numSegments; i++) {
			float step = 360f / numSegments;
			float a = i * step;
			Vector3 p1 = new Vector3((float)rotor.R*Mathf.Cos(a*Mathf.PI/180f), (float)rotor.R*Mathf.Sin((float)rotor.beta_0), (float)rotor.R*Mathf.Sin(a*Mathf.PI/180f));
			Vector3 p2 = new Vector3((float)rotor.R*Mathf.Cos((a+step)*Mathf.PI/180f), (float)rotor.R*Mathf.Sin((float)rotor.beta_0), (float)rotor.R*Mathf.Sin((a+step)*Mathf.PI/180f));
			Debug.DrawLine(transform.TransformPoint(p1), transform.TransformPoint(p2), Color.gray);
			if (i % (numSegments/rotor.Nb) == 0)
				Debug.DrawLine(transform.position, transform.TransformPoint(p1), Color.gray);

		}
	}

	private void DebugDrawStabilizer(Transform transform, Stabilizer stabilizer) {
		Vector3 tl = new Vector3((float)-stabilizer.span / 2, 0, (float)stabilizer.chord / 2);
		Vector3 tr = new Vector3((float)stabilizer.span / 2, 0, (float)stabilizer.chord / 2);
		Vector3 br = new Vector3((float)stabilizer.span / 2, 0, (float)-stabilizer.chord / 2);
		Vector3 bl = new Vector3((float)-stabilizer.span / 2, 0, (float)-stabilizer.chord / 2);
		Debug.DrawLine(transform.TransformPoint(tl), transform.TransformPoint(tr), Color.gray);
		Debug.DrawLine(transform.TransformPoint(tr), transform.TransformPoint(br), Color.gray);
		Debug.DrawLine(transform.TransformPoint(br), transform.TransformPoint(bl), Color.gray);
		Debug.DrawLine(transform.TransformPoint(bl), transform.TransformPoint(tl), Color.gray);
	}

	private float? GetHeight() {
		RaycastHit hit;
		if (Physics.Raycast(transform.position, new Vector3(0,-1,0), out hit, 1000f)) {
			return transform.position.y - hit.point.y;
		}
		return null;
	}

	public void LoadDefault() {
		model = new SingleMainRotorHelicopter().LoadDefault();
		FindComponents();
		ParametrizeUnityFromModel();
	}

	private void FindComponents() {
		body = GetComponent<Rigidbody>();
		if (body == null) {
			body = gameObject.AddComponent<Rigidbody>();
			body.mass = (float)model.Mass;
			body.drag = body.angularDrag = 0f;
		}
        Transform centerOfMassTransform = transform.FindChild("CenterOfMass");
        if (centerOfMassTransform != null) body.centerOfMass = centerOfMassTransform.localPosition;
	}

	private void ParametrizeUnityFromModel() {
		// Set mass and inertia
		body.mass = (float)model.Mass;
		Vector3 inertia = (model.Inertia * Vector<double>.Build.DenseOfArray(new double[] { 1, 1, 1 })).ToUnity();
		inertia.y *= -1; // all inertia tensor components must be positive
		body.inertiaTensor = inertia;
		Debug.Log("Inertia tensor " + body.inertiaTensor);
		// TODO what about body.inertiaTensorRotation ?

		// Sub models
		mainRotor = model.MainRotor;
		mainRotorTransform = transform.FindChild("MainRotor");
		if (mainRotorTransform == null) {
			GameObject mainRotorGO = new GameObject("MainRotor");
			mainRotorTransform = mainRotorGO.transform;
			mainRotorTransform.parent = transform;
		}
		mainRotorTransform.localPosition = model.MainRotor.Translation.ToUnity();
		mainRotorTransform.localRotation = model.MainRotor.Rotation.ToUnity();

		tailRotor = model.TailRotor;
		tailRotorTransform = transform.FindChild("TailRotor");
		if (tailRotorTransform == null) {
			GameObject tailRotorGO = new GameObject("TailRotor");
			tailRotorTransform = tailRotorGO.transform;
			tailRotorTransform.parent = transform;
		}
		tailRotorTransform.localPosition = model.TailRotor.Translation.ToUnity();
		tailRotorTransform.localRotation = model.TailRotor.Rotation.ToUnity();

		horizontalStabilizer = model.HorizontalStabilizer;
		horizontalStabilizerTransform = transform.FindChild("HorizontalStabilizer");
		if (horizontalStabilizerTransform == null) {
			GameObject horizontalStabilizerGO = new GameObject("HorizontalStabilizer");
			horizontalStabilizerTransform = horizontalStabilizerGO.transform;
			horizontalStabilizerTransform.parent = transform;
		}
		horizontalStabilizerTransform.localPosition = model.HorizontalStabilizer.Translation.ToUnity();
		horizontalStabilizerTransform.localRotation = model.HorizontalStabilizer.Rotation.ToUnity();

		verticalStabilizer = model.VerticalStabilizer;
		verticalStabilizerTransform = transform.FindChild("VerticalStabilizer");
		if (verticalStabilizerTransform == null) {
			GameObject verticalStabilizerGO = new GameObject("VerticalStabilizer");
			verticalStabilizerTransform = verticalStabilizerGO.transform;
			verticalStabilizerTransform.parent = transform;
		}
		verticalStabilizerTransform.localPosition = model.VerticalStabilizer.Translation.ToUnity();
		verticalStabilizerTransform.localRotation = model.VerticalStabilizer.Rotation.ToUnity();

		fuselage = model.Fuselage;
		fuselageTransform = transform.FindChild("Fuselage");
		if (fuselageTransform == null) {
			GameObject fuselageGO = new GameObject("Fuselage");
			fuselageTransform = fuselageGO.transform;
			fuselageTransform.parent = transform;
		}
		fuselageTransform.localPosition = model.Fuselage.Translation.ToUnity();
		fuselageTransform.localRotation = model.Fuselage.Rotation.ToUnity();

		fcs = model.FCS;
        engine = model.Engine;
        gearBox = model.GearBox;
	}

	private void ParametrizeModelsFromUnity() {
		model.LoadDefault ();
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
			model.MainRotor = mainRotor;
			model.MainRotor.Translation = mainRotorTransform.localPosition.FromUnity();
			model.MainRotor.Rotation = mainRotorTransform.localRotation.FromUnity();
		}
		tailRotorTransform = transform.FindChild("TailRotor");
		if (tailRotorTransform == null) {
			Debug.LogError("No TailRotor child");
		} else {
			model.TailRotor = tailRotor;
			model.TailRotor.Translation = tailRotorTransform.localPosition.FromUnity();
			model.TailRotor.Rotation = tailRotorTransform.localRotation.FromUnity();
		}
		horizontalStabilizerTransform = transform.FindChild("HorizontalStabilizer");
		if (horizontalStabilizerTransform != null) {
			model.HorizontalStabilizer = horizontalStabilizer;
			model.HorizontalStabilizer.Translation = horizontalStabilizerTransform.localPosition.FromUnity();
			model.HorizontalStabilizer.Rotation = horizontalStabilizerTransform.localRotation.FromUnity();
		}
		verticalStabilizerTransform = transform.FindChild("VerticalStabilizer");
		if (verticalStabilizerTransform != null) {
			model.VerticalStabilizer = verticalStabilizer;
			model.VerticalStabilizer.Translation = verticalStabilizerTransform.localPosition.FromUnity();
			model.VerticalStabilizer.Rotation = verticalStabilizerTransform.localRotation.FromUnity();
		}
		fuselageTransform = transform.FindChild("Fuselage");
		if (fuselageTransform != null) {
			model.Fuselage = fuselage;
			model.Fuselage.Translation = fuselageTransform.localPosition.FromUnity();
			model.Fuselage.Rotation = fuselageTransform.localRotation.FromUnity();
		}

		model.FCS = fcs;
        model.Engine = engine;
        model.GearBox = gearBox;
		model.Gravity.Enabled = false;
	}
}
