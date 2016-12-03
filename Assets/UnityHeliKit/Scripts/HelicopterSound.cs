using UnityEngine;
using System.Collections;

public class HelicopterSound : MonoBehaviour {

	public AudioSource[] rotorSounds;
	public AudioSource[] engineSounds;
	public AudioSource startupSound;
	public AudioSource shutdownSound;

	private Helicopter helicopter;
	private HeliSharp.Engine.Phase lastEnginePhase;

	void Start () {
		helicopter = GetComponent<Helicopter>();
	}

	void Update () {
		if (helicopter.isActiveAndEnabled) {
			foreach (var rotorSound in rotorSounds) {
				rotorSound.pitch = (float)(helicopter.model.MainRotor.RotSpeed / helicopter.model.MainRotor.designOmega);
			}
			foreach (var engineSound in engineSounds) {
				engineSound.pitch = (float)(helicopter.model.Engine.rotspeed / helicopter.model.Engine.Omega0);
			}
			if (helicopter.model.Engine.phase != lastEnginePhase) {
				switch (helicopter.model.Engine.phase) {
					case HeliSharp.Engine.Phase.START:
						if (startupSound != null && !startupSound.isPlaying) startupSound.Play();
						break;
					case HeliSharp.Engine.Phase.CUTOFF:
						if (shutdownSound != null && !shutdownSound.isPlaying) shutdownSound.Play();
						break;
				}
			}
			lastEnginePhase = helicopter.model.Engine.phase;
		} else {
			foreach (var sound in rotorSounds) sound.Stop ();
			foreach (var sound in engineSounds) sound.Stop ();
			if (startupSound != null) startupSound.Stop ();
			if (shutdownSound != null) shutdownSound.Stop ();
		}
	
	}
}
