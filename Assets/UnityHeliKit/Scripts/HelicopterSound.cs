using UnityEngine;
using System.Collections;

public class HelicopterSound : MonoBehaviour {

	public AudioSource[] rotorSounds;
    public AudioSource[] rotorSlapSounds;
    public AudioSource[] rotorDownwashSounds;
	public AudioSource[] engineSounds;
	public AudioSource startupSound;
	public AudioSource shutdownSound;
    public AudioSource windSound;

    public float maxWindSpeed = 50f;
    public float rotorSlapMinAngle = 0;
    public float rotorSlapMaxAngle = 10f;
    public float rotorDownwashMinVelocity = 0;
    public float rotorDownwashMaxVelocity = 10f;

    private Helicopter helicopter;
	private HeliSharp.Helicopter model;
	private HeliSharp.Engine.Phase? lastEnginePhase;

	void Start () {
		helicopter = GetComponent<Helicopter>();
	    if (helicopter != null) model = helicopter.model;
	}

	void Update () {
		if (helicopter.isActiveAndEnabled) {
			foreach (var rotorSound in rotorSounds) {
				rotorSound.pitch = (float)(model.Rotors[0].RotSpeed / model.Rotors[0].designOmega);
			}
            for (var i = 0; i < model.Rotors.Length; i++) {
                if (rotorSlapSounds.Length > i) {
                    rotorSlapSounds[i].pitch = (float)(model.Rotors[i].RotSpeed / model.Rotors[i].designOmega);
                    rotorSlapSounds[i].volume = Mathf.Clamp01(((float)model.Rotors[0].beta_0 * 180f / Mathf.PI - rotorSlapMinAngle) / (rotorSlapMaxAngle - rotorSlapMinAngle));
                }
                if (rotorDownwashSounds.Length > i) {
                    rotorDownwashSounds[i].volume = Mathf.Clamp01(((float)model.Rotors[0].WashVelocity.Norm(1) - rotorDownwashMinVelocity) / (rotorDownwashMaxVelocity - rotorDownwashMinVelocity));
                }
            }
			foreach (var engineSound in engineSounds) {
				engineSound.pitch = (float)(model.Engine.RotSpeed / model.Engine.Omega0);
			}
			if (lastEnginePhase.HasValue && model.Engine.phase != lastEnginePhase.Value) {
				switch (model.Engine.phase) {
					case HeliSharp.Engine.Phase.START:
						if (startupSound != null && !startupSound.isPlaying) startupSound.Play();
						break;
					case HeliSharp.Engine.Phase.CUTOFF:
						if (shutdownSound != null && !shutdownSound.isPlaying) shutdownSound.Play();
						break;
				}
			}
            if (windSound != null) {
                windSound.volume = Mathf.Clamp01((float)model.Fuselage.Velocity.Norm(1) / maxWindSpeed);
            }
			lastEnginePhase = model.Engine.phase;
		} else {
			foreach (var sound in rotorSounds) sound.Stop ();
			foreach (var sound in engineSounds) sound.Stop ();
			if (startupSound != null) startupSound.Stop ();
			if (shutdownSound != null) shutdownSound.Stop ();
		}
	
	}
}
