using UnityEngine;
using System.Collections;
using HeliSharp;

public class HelicopterInput : MonoBehaviour {

    public bool autoThrottle = true;
    public float throttleUpSpeed = 0.3f;
    public float throttleDownSpeed = 1f;
    public float autoThrottleWaitTime = 3f;

    private Helicopter helicopter;
    private float targetThrottle;

    enum AutoThrottleState {
        None,
        Start,
        ThrottleUp,
        Running,
        Shutdown,
        ThrottleDown,
        Cutoff,
    }
    private AutoThrottleState autoThrottleState;
    private float lastThrottleStateTime;

    void Start () {
        helicopter = GetComponent<Helicopter>();
        if (helicopter.airStart) targetThrottle = 1;
    }

    void Update () {
        UpdateAutoThrottle();

        if (!helicopter.playerControlled || helicopter.IsTrimming)
        {
            helicopter.Collective = helicopter.TrimControl ? 0f : -1f;
            helicopter.LongCyclic = helicopter.LatCyclic = helicopter.Pedal = 0f;
            return;
        }

        if (Input.GetButtonDown("ThrottleFull"))
            targetThrottle = 1;
        else if (Input.GetButtonDown("ThrottleHalf"))
            targetThrottle = 0.5f;
        else if (Input.GetButtonDown("ThrottleZero"))
            targetThrottle = 0;
        else if (Input.GetButtonDown("Engine")) {
            if (autoThrottle) {
                if (helicopter.engine.phase == Engine.Phase.CUTOFF)
                    autoThrottleState = AutoThrottleState.Start;
                else if (helicopter.IsOnGround && helicopter.engine.phase == Engine.Phase.RUN)
                    autoThrottleState = AutoThrottleState.Shutdown;
            } else {
                helicopter.ToggleEngine();
            }
        } else if (Input.GetButtonDown("Trim")) {
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
            helicopter.Collective = Input.GetAxis("CollectiveKey");
            helicopter.Pedal = Input.GetAxis("PedalKey");
        }
        if (helicopter.engine.phase == Engine.Phase.START) helicopter.Collective = -1;

        if (Input.GetButton("Brake"))
            helicopter.LeftBrake = helicopter.RightBrake = 1;
        else
            helicopter.LeftBrake = helicopter.RightBrake = 0;

    }

    void UpdateAutoThrottle() {
        switch (autoThrottleState) {
            case AutoThrottleState.Start:
                if (helicopter.engine.phase != Engine.Phase.RUN && helicopter.engine.phase != Engine.Phase.START) {
                    helicopter.Throttle = 0;
                    helicopter.ToggleEngine();
                } else if (helicopter.engine.phase == Engine.Phase.RUN) {
                    autoThrottleState = AutoThrottleState.ThrottleUp;
                    lastThrottleStateTime = Time.time;
                }
                break;
            case AutoThrottleState.ThrottleUp:
                if (helicopter.engine.phase == Engine.Phase.RUN && targetThrottle < 0.99f && Time.time - lastThrottleStateTime > autoThrottleWaitTime) {
                    targetThrottle = 1;
                } else if (helicopter.engine.RPM >= helicopter.engine.designRPM * 0.9) {
                    autoThrottleState = AutoThrottleState.Running;
                    lastThrottleStateTime = Time.time;
                }
                break;
            case AutoThrottleState.Shutdown:
                if (helicopter.engine.phase == Engine.Phase.RUN)
                    autoThrottleState = AutoThrottleState.ThrottleDown;
                else
                    autoThrottleState = AutoThrottleState.Cutoff;
                lastThrottleStateTime = Time.time;
                break;
            case AutoThrottleState.ThrottleDown:
                if (helicopter.engine.phase == Engine.Phase.RUN && targetThrottle > 0.01f) {
                    targetThrottle = 0;
                } if (helicopter.engine.RPM <= helicopter.engine.designRPM * helicopter.engine.idleRatio * 1.1) {
                    autoThrottleState = AutoThrottleState.Cutoff;
                    lastThrottleStateTime = Time.time;
                }
                break;
            case AutoThrottleState.Cutoff:
                if (helicopter.engine.phase != Engine.Phase.CUTOFF && Time.time - lastThrottleStateTime > autoThrottleWaitTime) {
                    helicopter.ToggleEngine();
                    autoThrottleState = AutoThrottleState.None;
                    lastThrottleStateTime = Time.time;
                }
                break;
        }

        if (Mathf.Abs(helicopter.Throttle - targetThrottle) > 1e-5f) {
            if (helicopter.Throttle < targetThrottle) {
                helicopter.Throttle += throttleUpSpeed * Time.deltaTime;
                if (helicopter.Throttle > targetThrottle) helicopter.Throttle = targetThrottle;
            } else {
                helicopter.Throttle -= throttleDownSpeed * Time.deltaTime;
                if (helicopter.Throttle < targetThrottle) helicopter.Throttle = targetThrottle;
            }
        }
    }
    
}
