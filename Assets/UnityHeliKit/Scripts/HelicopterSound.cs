using UnityEngine;
using System.Collections;

public class HelicopterSound : MonoBehaviour {

	public AudioSource[] rotorSounds;
	public AudioSource[] engineSounds;
	public AudioSource startupSound;
	public AudioSource shutdownSound;

    private Helicopter helicopter;
	private HeliSharp.Helicopter model;
	private HeliSharp.Engine.Phase lastEnginePhase;

	void Start () {
		helicopter = GetComponent<Helicopter>();
	    if (helicopter != null) model = helicopter.model;
	}

	void Update () {
		if (helicopter.isActiveAndEnabled) {
			foreach (var rotorSound in rotorSounds) {
				rotorSound.pitch = (float)(model.Rotors[0].RotSpeed / model.Rotors[0].designOmega);
			}
			foreach (var engineSound in engineSounds) {
				engineSound.pitch = (float)(model.Engine.rotspeed / model.Engine.Omega0);
			}
			if (model.Engine.phase != lastEnginePhase) {
				switch (model.Engine.phase) {
					case HeliSharp.Engine.Phase.START:
						if (startupSound != null && !startupSound.isPlaying) startupSound.Play();
						break;
					case HeliSharp.Engine.Phase.CUTOFF:
						if (shutdownSound != null && !shutdownSound.isPlaying) shutdownSound.Play();
						break;
				}
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
