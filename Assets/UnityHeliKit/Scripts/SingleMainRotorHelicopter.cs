using UnityEngine;
using HeliSharp;

public class SingleMainRotorHelicopter : Helicopter {

    public override HeliSharp.Helicopter model { get { return _model; } }
    public HeliSharp.SingleMainRotorHelicopter _model { get; set; }

    public Rotor mainRotor;
	public Rotor tailRotor;
	public Stabilizer horizontalStabilizer;
	public Stabilizer verticalStabilizer;
	public Fuselage fuselage;
    public Engine engine;
    public GearBox gearBox;

    public SingleMainRotorHelicopter() {
		_model = new HeliSharp.SingleMainRotorHelicopter ();
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

    public override void FixedUpdate () {
		if (body == null) return;

        base.FixedUpdate();

        if (debugText != null) {
			string text = "";
            text += _model.Engine.phase + " THR " + _model.Engine.throttle + " RPM E " + (_model.Engine.rotspeed * 9.5493).ToStr() + " M/R " + (_model.MainRotor.RotSpeed * 9.5493).ToStr() + "\n";
			text += "WASH " + Mathf.Round((float)_model.MainRotor.WashVelocity.Norm(2)) + "\n";
			debugText.text += text;
		}
    }

	public override void ParametrizeUnityFromModel() {
        base.ParametrizeUnityFromModel();
		mainRotor = _model.MainRotor;
		tailRotor = _model.TailRotor;
		horizontalStabilizer = _model.HorizontalStabilizer;
		verticalStabilizer = _model.VerticalStabilizer;
		fuselage = _model.Fuselage;
        engine = _model.Engine;
        gearBox = _model.GearBox;
	}

	public override void ParametrizeModelsFromUnity() {
	    model.LoadDefault();
	    _model.MainRotor = mainRotor;
	    _model.TailRotor = tailRotor;
	    _model.HorizontalStabilizer = horizontalStabilizer;
	    _model.VerticalStabilizer = verticalStabilizer;
	    _model.Fuselage = fuselage;
	    _model.Engine = engine;
	    _model.GearBox = gearBox;
	    base.ParametrizeModelsFromUnity();
	}
}
