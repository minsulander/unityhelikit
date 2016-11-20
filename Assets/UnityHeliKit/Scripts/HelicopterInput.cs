using UnityEngine;
using System.Collections;

public class HelicopterInput : MonoBehaviour {

    private Helicopter helicopter;

    void Start () {
        helicopter = GetComponent<Helicopter>();
    }

    void FixedUpdate () {
        if (Input.GetKeyDown("1"))
            helicopter.Throttle = 1;
        else if (Input.GetKeyDown("5"))
            helicopter.Throttle = 0.5f;
        else if (Input.GetKeyDown("0"))
            helicopter.Throttle = 0;
        else if (Input.GetButtonDown("Engine"))
            helicopter.ToggleEngine();
        else if (Input.GetButtonDown("Trim")) {
            helicopter.Trim(false);
            return;
        }

        if (Input.GetJoystickNames().Length > 0) {
            helicopter.Collective = Input.GetAxis("Collective");
            helicopter.LongCyclic = Input.GetAxis("LongCyclic");
            helicopter.LatCyclic = Input.GetAxis("LatCyclic");
            helicopter.Pedal = Input.GetAxis("Pedal");
        } else {
            helicopter.LongCyclic = Input.GetAxis("Vertical");
            helicopter.LatCyclic = Input.GetAxis("Horizontal");
        }

        if (Input.GetButton("Brake"))
            helicopter.LeftBrake = helicopter.RightBrake = 1;
        else
            helicopter.LeftBrake = helicopter.RightBrake = 0;
    }
}
