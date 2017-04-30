using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using HeliSharp;
using MathNet.Numerics.LinearAlgebra;

public abstract class Helicopter : MonoBehaviour
{

	public bool airStart = true;
	public Text debugText;

	public FlightControlSystem fcs;
    public Engine engine;
    public GearBox gearBox;
    public Fuselage fuselage;

    public abstract HeliSharp.Helicopter model { get; }

    protected Rigidbody body;
    protected Dictionary<string, Transform> submodelTransforms = new Dictionary<string, Transform>();

    private object trimThreadLock = new object();
    private System.Threading.Thread trimThread = null;
    private bool trimDone = false;
    public bool IsTrimming { get {
            lock (trimThreadLock) {
                return trimThread != null && !trimDone;
            }
        }
    }

    // "Relay" controls to dynamics model
    public float Throttle {
        get { return (float)model.Engine.throttle; }
        set { model.Engine.throttle = value; }
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

    private Dictionary<Rotor, float> rotorSpinAngle = new Dictionary<Rotor, float>();

    public virtual void Start () {
        FindComponents();
        ParametrizeModelsFromUnity();
        Trim(true);
    }

    public virtual void FindComponents() {
        body = GetComponent<Rigidbody>();
        if (body == null) {
            body = gameObject.AddComponent<Rigidbody>();
            body.mass = (float)model.Mass;
            body.drag = body.angularDrag = 0f;
        }
        Transform centerOfMassTransform = transform.FindChild("CenterOfMass");
        if (centerOfMassTransform != null) body.centerOfMass = centerOfMassTransform.localPosition;
        foreach (var submodelName in model.SubModels.Keys) {
            if (submodelName == "Gravity") continue;
            var submodel = model.SubModels[submodelName];
            var childTransform = transform.FindChild(submodelName);
            if (childTransform == null) {
                var childObject = new GameObject(submodelName);
                childTransform = childObject.transform;
                childTransform.parent = transform;
            }
            submodelTransforms[submodelName] = childTransform;
        }
    }

    public virtual void Trim(bool initial)
    {
        Debug.Log("Trim" + (initial ? " initial" : ""));

        Quaternion rotation = body.rotation;
        body.useGravity = false;

        lock (trimThreadLock) {
            trimDone = false;
            trimThread = new System.Threading.Thread(delegate () {
                model.Rotation = rotation.FromUnity();
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
                }
                lock (trimThreadLock) {
                    trimDone = true;
                }
                if (initial) model.InitEngine(airStart);
            });
            trimThread.Start();
        }

    }

    public void ToggleEngine() {
        if (model.Engine.phase == Engine.Phase.CUTOFF) {
           model.Engine.phase = Engine.Phase.START;
        } else if (model.Engine.phase == Engine.Phase.RUN) {
            model.Engine.phase = Engine.Phase.CUTOFF;
        }
    }

    public virtual void FixedUpdate() {
        if (body == null) return;
        lock (trimThreadLock) {
            if (trimThread != null) {
                if (trimDone) {
                    Debug.Log("Trim done");
                    body.useGravity = true;
                    body.rotation = model.Rotation.ToUnity();
                    trimThread = null;
                } else {
                    return;
                }
            }
        }

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
            text += "AVEL " + (int)(model.AngularVelocity.x() * 100) + " " + (int)(model.AngularVelocity.y() * 100) + " " + (int)(model.AngularVelocity.z() * 100) + "\n";
            //text += "F " + (int)model.Force.x() + " " + (int)model.Force.y() + " " + (int)model.Force.z() + "\n";
            //text += "M " + (int)model.Torque.x() + " " + (int)model.Torque.y() + " " + (int)model.Torque.z() + "\n";
            //text += "M/R F " + (int)mainRotor.Force.x() + " " + (int)mainRotor.Force.y() + " " + (int)mainRotor.Force.z() + "\n";
            //text += "M/R M " + (int)mainRotor.Torque.x() + " " + (int)mainRotor.Torque.y() + " " + (int)mainRotor.Torque.z() + "\n";
            //text += "H/S Fz " + (int)horizontalStabilizer.Force.z () + " V/S Fz " + (int)verticalStabilizer.Force.z () + "\n";
            //text += "FUSE Mz " + (int)fuselage.Torque.z () + "\n";
            //text += "uF " + (int)force.x + " " + (int)force.y + " " + (int)force.z + "\n";
            //text += "uM " + (int)torque.x + " " + (int)torque.y + " " + (int)torque.z + "\n";
            text += "WASH " + (int)model.Rotors[0].WashVelocity.Norm(1) + " CONE " + (int)(model.Rotors[0].beta_0 * 180 / Mathf.PI) + "\n";
            text += model.Engine.phase + " THR " + model.Engine.throttle + " RPM E " + model.Engine.RPM.ToStr() + " RPM R " + model.Rotors[0].RPM.ToStr() + "\n";
            if (LeftBrake > 0.01f || RightBrake > 0.01f) text += "BRAKE\n";
            debugText.text = text;
        }

        // Update dynamics
		try {
        	model.Update(Time.fixedDeltaTime);
		} catch (ModelException e) {
			Debug.LogException(e);
			enabled = false;
			return;
		}

        // Set force and torque/moment in rigid body simulation
        Vector3 force = model.Force.ToUnity();
        Vector3 torque = -model.Torque.ToUnity(); // minus because Unity uses a left-hand coordinate system
        body.AddRelativeForce(force);
        body.AddRelativeTorque(torque);

        // Spin rotors
        foreach (var submodelName in submodelTransforms.Keys) {
            var submodel = model.SubModels[submodelName];
            var childTransform = submodelTransforms[submodelName];
            if (submodel is Rotor) {
                if (double.IsNaN(((Rotor)submodel).RotSpeed)) {
                    Debug.LogError(name + ": rotor speed is NaN");
                    enabled = false;
                    return;
                }
                SpinRotor(childTransform, (Rotor)submodel);
            }
        }
    }

    public float? GetHeight() {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, new Vector3(0,-1,0), out hit, 1000f)) {
            return transform.position.y - hit.point.y;
        }
        return null;
    }

    public virtual void ParametrizeUnityFromModel() {
        // Set mass and inertia
        body.mass = (float)model.Mass;
        Vector3 inertia = (model.Inertia * Vector<double>.Build.DenseOfArray(new double[] { 1, 1, 1 })).ToUnity();
        inertia.y *= -1; // all inertia tensor components must be positive
        body.inertiaTensor = inertia;
        Debug.Log("Inertia tensor " + body.inertiaTensor);
        // TODO what about body.inertiaTensorRotation ?

        fcs = model.FCS;
        engine = model.Engine;
        gearBox = model.GearBox;
        fuselage = model.Fuselage;

        foreach (var submodelName in submodelTransforms.Keys) {
            var submodel = model.SubModels[submodelName];
            var childTransform = submodelTransforms[submodelName];
            Debug.Log("Transform from submodel: " + childTransform.name);
            childTransform.localPosition = submodel.Translation.ToUnity();
            childTransform.localRotation = submodel.Rotation.ToUnity();

			if (submodel is Rotor) {
				BoxCollider collider = childTransform.GetComponent<BoxCollider> ();
				var sizey = 0.1f;
				if (collider == null) {
					collider = childTransform.gameObject.AddComponent<BoxCollider> ();
				} else {
					sizey = collider.size.y;
				}
				var radius = (float)((Rotor)submodel).radius;
				var size = Mathf.Sqrt (2 * radius * radius);
				collider.size = new Vector3 (size, sizey, size);
			}
		}
    }

    public virtual void ParametrizeModelsFromUnity() {
        //model.LoadDefault();
        // Set mass and inertia
        //model.Mass = body.mass;
        //model.Inertia = Matrix<double>.Build.DenseOfDiagonalVector(body.inertiaTensor.FromUnity());
        //Debug.Log(name + " mass " + model.Mass.ToStr());
        //Debug.Log(name + " inertia " + model.Inertia.ToStr());
        // TODO what about body.inertiaTensorRotation ?

        model.FCS = fcs;
        model.Engine = engine;
        model.GearBox = gearBox;
        model.Fuselage = fuselage;
        model.Gravity.Enabled = false;

        foreach (var submodelName in submodelTransforms.Keys) {
            var submodel = model.SubModels[submodelName];
            var childTransform = submodelTransforms[submodelName];
            Debug.Log("Submodel from transform: " + childTransform.name);
            submodel.Translation = childTransform.localPosition.FromUnity();
            submodel.Rotation = childTransform.localRotation.FromUnity();
        }
    }

    protected void SpinRotor(Transform rotorTransform, Rotor rotor) {
        if (!rotorSpinAngle.ContainsKey(rotor)) rotorSpinAngle[rotor] = 0f;
		rotorSpinAngle[rotor] += (rotor.direction == Rotor.Direction.CounterClockwise ? -1 : 1) * (float)rotor.RotSpeed * 180f / Mathf.PI * Time.deltaTime;
        while (rotorSpinAngle[rotor] > 360f) rotorSpinAngle[rotor] -= 360f;
        rotorTransform.localRotation = Quaternion.Euler(new Vector3((float)rotor.beta_cos * 180f / Mathf.PI, 0, (float)rotor.beta_sin * 180f / Mathf.PI)) * rotor.Rotation.ToUnity()
            * Quaternion.AngleAxis(rotorSpinAngle[rotor], new Vector3(0, 1, 0));
    }

	void OnCollisionEnter(Collision collision) {
		foreach (var contact in collision.contacts) {
			if (submodelTransforms.ContainsValue (contact.thisCollider.transform)) {
				var submodelName = contact.thisCollider.transform.name;
				var submodel = model.SubModels[submodelName];
				if (submodel is Rotor && submodel.Enabled) {
					Debug.Log ("Disabling " + submodelName + " due to collision");
					contact.thisCollider.enabled = false;
					submodel.Enabled = false;
				}
			}
		}
	}

    public virtual void OnDrawGizmosSelected() {
        if (model == null) return;
        float visualizationScale = (float)(model.Rotors[0].radius / model.Mass / 9.81);
        foreach (var submodelName in submodelTransforms.Keys) {
            var submodel = model.SubModels[submodelName];
            var childTransform = submodelTransforms[submodelName];
            Debug.DrawLine(transform.TransformPoint(submodel.Translation.ToUnity()), transform.TransformPoint(submodel.Translation.ToUnity() + (submodel.Rotation * submodel.Force).ToUnity() * visualizationScale), Color.yellow);
            if (submodel is Rotor) {
                Rotor rotor = (Rotor) submodel;
                DebugDrawRotor(childTransform, rotor, 32);
                Debug.DrawLine(childTransform.position, childTransform.TransformPoint(-rotor.WashVelocity.ToUnity() / 3), Color.blue);
            }
        }
    }

    protected void DebugDrawRotor(Transform rotorTransform, Rotor rotor, int numSegments) {
        for (int i = 0; i < numSegments; i++) {
            float step = 360f / numSegments;
            float a = i * step;
            Vector3 p1 = new Vector3((float)rotor.radius * Mathf.Cos(a * Mathf.PI / 180f), (float)rotor.radius * Mathf.Sin((float)rotor.beta_0), (float)rotor.radius * Mathf.Sin(a * Mathf.PI / 180f));
            Vector3 p2 = new Vector3((float)rotor.radius * Mathf.Cos((a + step) * Mathf.PI / 180f), (float)rotor.radius * Mathf.Sin((float)rotor.beta_0), (float)rotor.radius * Mathf.Sin((a + step) * Mathf.PI / 180f));
            Debug.DrawLine(rotorTransform.TransformPoint(p1), rotorTransform.TransformPoint(p2), Color.gray);
            if (i % (numSegments / rotor.bladeCount) == 0)
                Debug.DrawLine(rotorTransform.position, rotorTransform.TransformPoint(p1), Color.gray);

        }
    }

    protected void DebugDrawStabilizer(Transform transform, Stabilizer stabilizer) {
        Vector3 tl = new Vector3((float)-stabilizer.span / 2, 0, (float)stabilizer.chord / 2);
        Vector3 tr = new Vector3((float)stabilizer.span / 2, 0, (float)stabilizer.chord / 2);
        Vector3 br = new Vector3((float)stabilizer.span / 2, 0, (float)-stabilizer.chord / 2);
        Vector3 bl = new Vector3((float)-stabilizer.span / 2, 0, (float)-stabilizer.chord / 2);
        Debug.DrawLine(transform.TransformPoint(tl), transform.TransformPoint(tr), Color.gray);
        Debug.DrawLine(transform.TransformPoint(tr), transform.TransformPoint(br), Color.gray);
        Debug.DrawLine(transform.TransformPoint(br), transform.TransformPoint(bl), Color.gray);
        Debug.DrawLine(transform.TransformPoint(bl), transform.TransformPoint(tl), Color.gray);
    }

}
