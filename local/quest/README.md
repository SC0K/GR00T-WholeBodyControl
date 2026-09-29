# Quest Pro → SONIC simulation (experimental local adapter)

This adapter connects Quest Browser controller or hand tracking to the **SC0K GR00T-WholeBodyControl**
checkout. It uses SONIC's existing `zmq_manager` VR_3PT / teleop encoder. No APK,
PICO SDK, CloudXR, or changes to xr_teleoperate are needed.

## Start (three terminals)

Run each command from `~/GR00T-WholeBodyControl`:

1. `./local/start-sim.sh`
2. `./local/start-quest-controller.sh` — wait for `Init Done`.
3. `./local/start-quest.sh` — wait for `Quest HTTPS ready`.

The simulator holds the robot during initialization, then automatically releases
the suspension when SONIC publishes control feedback. **Do not press `9`** with
these launchers: that key would re-enable the suspension. DDS uses loopback and
ZMQ uses localhost. This adapter is for simulation only.

## Connect the Quest

1. Put the Quest and this PC on the same Wi-Fi/network.
2. Pick up both Quest controllers. The page now uses controllers only;
   bare hands and physical headset-following are not selected.
3. Open **Quest Browser** at **https://10.42.0.1:8013** when connected to the PC hotspot.
   If the network changes, run `hostname -I`
   and substitute the PC's address reachable from the headset.
4. The server uses a local self-signed certificate. If the browser shows a
   certificate interstitial, use its Advanced/Proceed option for this local
   address. If the browser disallows proceeding or WebXR remains unavailable,
   a certificate trusted by the headset is needed; do not disable browser security.
5. Tap **Enter passthrough** and allow XR access. Both controllers and the head
   must remain tracked. **Enter VR instead** uses a black background.
6. **Hold both thumbstick clicks for one second**, then release them. Alternatively,
   in the **Quest bridge terminal**, type `go` then Enter. This starts the
   standing planner first. The simulator prints `suspension released automatically`
   and the robot settles onto the ground. Keep both controllers tracked, stand
   upright, look forward, and hold your arms low to approximately match the robot’s standing arm pose,
   triggers released. Do not start with forearms horizontal if the robot’s arms are down. Hold still until calibration finishes. Calibration waits at least five seconds, at least two seconds
   of controller feedback, one second of continuous tracking, and fresh robot state.
   It cancels after 20 seconds if these conditions are not met, leaving standing
   control active. Type `p` to cancel/pause, or `o` for emergency stop.
7. Check for `VR 3-point control enabled` and encoder `teleop` (ID 1) in
   the controller terminal. The robot should be on the ground. If it shakes
   violently, use `o` and reset; do not leave it hanging under active control.

Headset page: passthrough and tracking only. **Watch the simulated robot in the
PC's MuJoCo window.** A robot camera/3D scene stream into the headset is not included.
There is no in-headset calibration panel yet; use the controller shortcut or terminal.

## Controller controls and calibration

- Move controllers to move the corresponding arms; index triggers close/open hands.
- **Left thumbstick:** forward/backward and sideways walking relative to robot heading.
- **Right thumbstick horizontal:** turn (up to 0.6 rad/s).
- Walking speed is capped at 0.35 m/s, acceleration at 0.5 m/s², with a 20% stick deadzone.
- **Hold both thumbstick clicks for one second:** request calibration/start when
  paused, or pause/cancel when active/starting. Walking input is suppressed during the chord.
- Headset movement does not command walking or turning. The head still supplies
  a reference for arm mapping. This is not foot tracking or full-body retargeting.

For calibration, stand upright and look forward. Match the robot’s current low
standing arm pose with your controllers, keeping triggers released. Do not use
a fixed 90-degree elbow pose when the robot’s arms are down. Release both
thumbstick clicks after requesting start, center the sticks, and hold still
through the five-second delay. The robot first returns to neutral standing;
calibration requires two seconds at the standing target, fresh controller/state
feedback, continuous tracking, and centered thumbsticks. The neutral controller
pose maps to the robot's current wrist poses; subsequent arm movements are
relative offsets. Calibration does not lift the robot arms: after it finishes,
slowly raise your controllers to bring the robot wrists up.

## Connection-loss recovery

Tracking or headset-websocket loss pauses teleop, clears calibration and walking
commands, opens the hands, disables crouch mapping, and returns to standing at
0.78 m target height with a bent-elbow ready pose (upper arms down, forearms
forward, roughly 90 degrees). The bridge generates these arm targets locally
from the G1 model, with position/rotation rate limits, rather than replaying
stale controller poses. Pause uses the same recovery pose. It restores a neutral posture at the
robot's current location/heading; it does not teleport the robot back to its
spawn position. The return from crouching is gradual.

The browser reconnects automatically (1–5 second retries) without ending the XR
session. **It never automatically resumes teleop.** Once both controllers are
tracked again, release **both** thumbstick clicks, then hold both for one second.
Wait for the robot’s arms to settle, match its bent-elbow ready pose, center
the sticks, and wait for recalibration. On the first start the standing arms
may still be low: match the actual robot pose rather than assuming a 90-degree
bend. After an active session has paused, the new ready-arm fallback is used.
A chord held across the outage cannot start control. Unrelated hand input-source
changes are ignored in controller mode. Use `go` in the terminal as a fallback.

## Controls (Quest bridge terminal, each followed by Enter)

| Key | Action |
| --- | --- |
| `go` | Start standing control, then calibrate/start teleop after the readiness delay; also explicitly resumes after tracking loss |
| `c` | Manually calibrate while teleop is paused and standing control is ready |
| `r` | Start calibrated VR input |
| `p` | Pause teleop and keep the standing planner active |
| `o` | Emergency stop; exits the SONIC controller |
| `s` | Show tracking, frame count, calibration, and controller-feedback status |
| `h` | Legacy crouch toggle; disabled in controller-only walking mode |
| `q` | Stop and close the bridge |

Wrist targets use a 120 ms smoothing time constant and rate limits of 0.4 m/s
and 1.5 rad/s. Pinch is smoothed too. The torso stays at its calibrated pose;
head rotation no longer commands torso tilt. These are conservative mitigations,
not a validation of free-standing policy stability.

Triggers close each simulated Dex3 hand through a grasp synergy, not individual
finger retargeting. Release the trigger to open.

Simulation-state or controller-feedback loss, `o`, `q`, and bridge shutdown
still send the emergency stop, which exits the controller. Restart the processes
after an emergency stop. Stop the bridge first, then the other terminals.
Standing and ready-arm fallback depend on the bridge continuing to run; it is not a separate
hardware watchdog.

## Validation and limitations

24 Python tests and two Node test scripts pass. Regression tests cover controller grips/triggers/axes, motion limits, neutral
standing reset, centered-stick calibration, explicit resume, and real local
websocket disconnect/reconnect. Browser tests verify that reconnection preserves
XR, sends no automatic start, and requires a new button release/hold. Tests use
mock controller input; real headset radio/tracking performance is not established.

The controller-only update also passed a 23-second MuJoCo + SONIC test with
synthetic controller arms, thumbstick walking/turning, tracking loss, standing
reset and explicit recalibration/resume. No falls were logged. Results:
`.local/quest-sticks-test.log`, `.local/quest-sticks-sim.log`, and
`.local/quest-sticks-controller.log`.

The bent-elbow disconnect fallback passed a MuJoCo + SONIC test with synthetic
controller input: tracking loss, gradual ready-arm recovery, explicit calibration
and walking/turning resume. Wrist target errors after recovery were 1.6 cm left
and 0.9 cm right; no falls were logged. Logs: `.local/quest-ready-arms-test.log`,
`.local/quest-ready-arms-sim.log`, `.local/quest-ready-arms-controller.log`.
This does not validate real headset tracking reliability.

The earlier ground test used synthetic tracking with the actual MuJoCo + SONIC
processes: automatic suspension release, teleop, standing pause and explicit
resume, with no falls logged (`.local/quest-ground-*.log`).

Actual Quest controller tracking and free-standing motion still require headset
validation. Wrists are mapped relative to the head, and torso orientation is
held at calibration. No headset robot-camera stream or full-body reconstruction
is included. Network recovery requires the bridge/controller processes to stay
running; restarting either process requires a new startup.

## Maintenance

Extra Python dependency: `aiohttp==3.13.3` in `.venv_sim`. All other bridge
requirements are already in the simulator environment.

```bash
.tools/bin/uv pip install --python .venv_sim/bin/python aiohttp==3.13.3
.venv_sim/bin/python -m unittest discover -s local/quest -p 'test_*.py' -v
node --check local/quest/quest.js
node local/quest/test_input.js
node local/quest/test_connection.js
```

The local certificate/key are in `.local/quest-certs/` (excluded from git).
Bridge source files are in `local/quest/`; automatic suspension release is in
`gear_sonic/utils/mujoco_sim/auto_release.py`. No native controller rebuild is required.
Restart all three processes after updating. `local/start-sim.sh --no-auto-release-suspension`
restores manual suspension control for diagnostics.

Protocol references:
- Local upstream docs: `docs/source/tutorials/vr_wholebody_teleop.md`.
- [WebXR Device API](https://www.w3.org/TR/webxr/).
- [WebXR controller/gamepad mapping](https://www.w3.org/TR/webxr-gamepads-module-1/).
- [WebXR Hand Input](https://www.w3.org/TR/webxr-hand-input-1/).
