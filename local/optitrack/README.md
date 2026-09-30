# OptiTrack full-body SONIC teleoperation (MuJoCo)

The local adapter receives `Skeleton 001` from Motive, calibrates its bone axes,
converts full-body rotations to SONIC SMPL features, and publishes protocol-v3
pose windows at 50 Hz. Both Dex3 hands stay open and wrist joint targets stay
neutral. All robot communication uses loopback simulation.

## Verified connection

- Motive `3.3.4.1`, advertised NatNet `4.2.0.0`.
- Motive PC: `192.168.0.118`; Linux: `eno1`, `192.168.0.17`.
- Unicast, command port `1510`, data port `1511`.
- Skeleton: `Skeleton 001`, ID `0`, 51 described and received bones.
- Global coordinates, meters, Z-up geometry; FBX-style bone names.
- The existing NatNet Linux SDK `4.0.0.0` is used with a requested 4.0 bitstream.

Skeleton streaming was disabled in Motive. It was enabled through NatNet's
`SetProperty,,Skeletons,true` command and verified with the getter and incoming
frames. The receiver itself does not change Motive settings. If streaming stops,
check Motive's Skeletons option and the active skeleton. See the official
[streaming settings](https://docs.optitrack.com/motive/data-streaming) and
[NatNet commands](https://docs.optitrack.com/developer-tools/natnet-sdk/natnet-remote-requests-commands).

## Launch

Run these in **three terminals**, from the repository root. Stop any Quest
bridge first: only one publisher can bind port 5556.

Terminal 1 — MuJoCo:

```bash
./local/start-mocap-sim.sh
```

Terminal 2 — SONIC controller:

```bash
./local/start-mocap-controller.sh
```

Wait for **`Init Done`** before requesting control.

Terminal 3 — mocap bridge:

```bash
./local/start-mocap.sh
```

Commands are entered in **terminal 3**, followed by Enter:

1. Hold a **T-pose**: stand upright with legs straight, arms straight out sideways
   and level, palms down. Type `c` and hold still for the one-second capture.
   Calibration is rejected if the pose is unsuitable, moving, or incomplete.
2. Lower your arms into a relaxed standing pose. Type `s` to start standing control.
   The simulator releases its suspension automatically on controller feedback.
   Do not press `9`, which would toggle the suspension again.
3. Wait at least five seconds. Match the simulated robot's standing pose, then
   press Enter on an empty line to enable full-body tracking.
4. `p` pauses and requests standing; an empty Enter explicitly resumes after
   you match the standing posture again.
5. `o` stops the controller. `q` exits the bridge and sends stop commands.
   Restart terminal 2 after a stop before starting another control session.

Calibration is saved in `.venv_teleop/mocap/calibration.json`. **An actual subject
T-pose calibration and human-motion trial remain to be performed.** Do not use
a calibration from a different actor, skeleton definition, or Motive ground frame.
After loss of skeleton data or an abrupt rotation, recalibration is required:
stop with `o`, calibrate with `c`, restart the controller, then start again.

The controller and launchers are configured for simulation; they provide no
real-robot network option. Full-body human tracking and free-standing stability
have not been validated by the suspended integration smoke tests.

## Receive-only diagnostics

```bash
./local/optitrack/probe.sh
./local/start-mocap.sh --monitor 5
./local/start-mocap.sh --calibrate   # T-pose capture only; no robot publication
```

The probe prints SDK version, bone names/IDs/hierarchy, and skeleton frame counts.
It exits 0 if skeleton frames arrive, 3 if descriptions/frames are missing, 1 on
connection failure, and 2 for configuration/build prerequisites. Output is in
`.venv_teleop/mocap/probe.log`.

Tracking checks cover missing bones, nonfinite poses, invalid quaternion norms,
bone lengths/global coordinates, fresh frames, and frame-counter progression.
This Motive stream reports zero per-bone flags, so the adapter does not claim
per-bone optical tracking confidence or distinguish measured from inferred poses.

## Mapping and runtime behavior

`retarget.py` defines the body mapping. Motive's two spine segments map to SMPL's
spine1/spine2; spine3 inherits spine2, so its local rotation is identity. The
remaining hips, legs, feet, neck, head, collars, arms and wrists map by name.
Finger bones are ignored. Calibration aligns each source bone's reference axes
with a common SMPL T-pose basis, and retains heading derived from the hip line.
SONIC's existing `process_smpl_joints` computes its canonical 24 joint features;
raw Motive positions are not substituted for SMPL joint positions.

The publisher fills a 15-frame buffer before switching from the standing planner
to pose tracking. Source loss/staleness beyond 0.35 seconds or an abrupt rotation
pauses pose tracking, clears calibration, and requests zero-speed standing with
open hands. Fresh tracking alone never resumes control. Loss of robot state or
controller feedback requests a stop. No Quest-trigger or finger-tracking input
is integrated.

`setup.json` stores connection/skeleton selection and documents the fixed runtime
policy. The adapter reads the connection, skeleton ID and simulation-only setting;
other fields describe behavior implemented in code, rather than arbitrary runtime
options. `--calibration` can override the calibration file location.

## Installed native runtime

The `.venv_teleop/mocap/native` directory contains TensorRT `10.13.3.9`, isolated
CUDA `12.9` headers/runtime/compiler, and the CMake build. ONNX Runtime `1.16.3`
is reused from `/opt/onnxruntime`. SONIC release encoder/decoder and V2 planner
models were downloaded using `download_from_hf.py`; TensorRT caches were generated
for this RTX 4090. The controller launcher prioritizes the bundled Unitree DDS
libraries to avoid an incompatible mixture with ROS DDS.

Rebuild the installed components:

```bash
./local/optitrack/build-receiver.sh
./local/optitrack/build-controller.sh
```

The NatNet SDK is reused from:
`/home/sitongchen/keyLM_ros2_ws/src/crl-humanoid-ros/crl_optitrack_ros/optitrack_adaptor/ext/NatNetSDK_linux`.
Set `NATNET_SDK_ROOT` when rebuilding against another SDK location.

Tests and logs:

```bash
PYTHONPATH="$PWD/local/optitrack" .venv_teleop/bin/python -m pytest -q local/optitrack/test_mocap.py
```

Logs, live diagnostic samples, and calibration files are in the ignored
`.venv_teleop/mocap/` directory. Tests cover calibration axes, heading, lower-body
rotation, full-buffer startup, open-hand messages, stale/missing data, and explicit
resume. Synthetic SMPL messages were accepted by the native controller in a
suspended MuJoCo smoke test with finite state feedback. This is not a human-motion
or free-standing balance validation.

Final suspended integration check: 347 standing, 250 SMPL-tracking, and 150
return-to-standing feedback samples were finite; shutdown completed without a
planner timeout. A native command-handler fix makes stop return immediately
instead of falling through into planner reinitialization. All 15 mocap unit
tests pass. Validation processes were stopped after testing.
