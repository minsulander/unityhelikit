using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FailureTest : MonoBehaviour {

	private SingleMainRotorHelicopter heli;

	// Use this for initialization
	void Start () {
		heli = GetComponent<SingleMainRotorHelicopter> ();
	}
	
	// Update is called once per frame
	void Update () {
		if (Input.GetKeyDown (KeyCode.Backspace)) {
			heli.tailRotor.Enabled = false;
		}
	}
}
