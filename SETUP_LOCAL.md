# Local SONIC simulation setup

Repository: https://github.com/SC0K/GR00T-WholeBodyControl
Revision: b042411fae38ee4d1af9aac82a37a1f8d14d6dd0
Location: /home/sitongchen/GR00T-WholeBodyControl

This installation is separate from ~/xr_teleoperate. It provides the pretrained
SONIC whole-body controller and G1 MuJoCo simulator, including the hand model.
Training dependencies, MotionBricks, and SMPL assets are not installed.

## Start

In terminal 1:

```bash
cd ~/GR00T-WholeBodyControl
./local/start-sim.sh
```

In terminal 2:

```bash
cd ~/GR00T-WholeBodyControl
./local/start-controller.sh
```

Wait for controller initialization, then press `]` in the controller terminal.
The local simulator launcher automatically releases the suspension when control
feedback arrives. Do not press `9`: it would re-enable the suspension.
In the controller terminal, press `T` to play a reference motion; `N`/`P`
select the next/previous reference. Press `O` to stop control. Stop both
processes with Ctrl+C in their respective terminals.

The controller launcher always uses loopback `lo` and includes the upstream
`--disable-crc-check` option for simulated state messages. Use these launchers
for simulation. The upstream `deploy.sh` defaults to real hardware when called
without `sim`.

## Installed components

- `.venv_sim`: Python 3.10, editable gear_sonic[sim] and Unitree SDK,
  MuJoCo, CPU PyTorch 2.7.0 (GPU policy inference is in the C++ controller).
- `.tools`: isolated Python/uv tools.
- `.native`: isolated CUDA 12.9 compiler/runtime and native build dependencies.
- `.local/TensorRT`: TensorRT 10.13.3.9 for CUDA 12.9.
- `.local/onnxruntime-linux-x64-1.16.3`: ONNX Runtime C++ library.
- Default pretrained encoder, decoder, observation configuration, and V2
  planner downloaded using the repository's download_from_hf.py.
- Required robot/reference Git LFS assets downloaded.

Rebuild the controller with `./local/build-controller.sh`.
Dependency snapshot: `.local/sim-packages.txt`.
Build and verification logs are in `.local/`.
TensorRT caches are generated for this machine's RTX 4090.

## Quest Pro adapter

An experimental **Quest Browser → SONIC VR_3PT** adapter is installed.
See [Quest connection instructions](local/quest/README.md) for the three-terminal
launch sequence, headset URL, calibration, and controls. Use
`local/start-quest-controller.sh` for this workflow, not the keyboard launcher.

Synthetic tracking was verified through HTTPS/WebSocket, calibration, ZMQ,
and the SONIC teleop encoder in suspended simulation. Real Quest tracking and
free-standing motion still need headset validation. This provides controller arm tracking, trigger grasp, thumbstick walking/turning, and
standing reset with gradual bent-elbow ready arms and explicit recalibration
after tracking/connection loss. After a pause, wait for the robot arms to settle
and match that pose before recalibrating. It
does not implement foot tracking, full-body reconstruction, or a headset robot-view stream.

## Verification performed

- Python dependency check passed; C++ controller built successfully.
- MuJoCo loaded the G1/hand assets and published live DDS state on loopback.
- Decoder, encoder, and planner TensorRT engines initialized and cached.
- Pressing `]` entered CONTROL; repeated logs showed fresh robot state and
  successful policy inference (roughly 0.1–0.3 ms in this short test).
- This was a suspended, headless control-loop smoke test, not a validation of
  free-standing balance, reference-motion playback, or VR teleoperation.
- Verification processes were stopped after testing.

The upstream simulator may print a caught channel initialization warning because
it initializes the same DDS domain twice. The live DDS/control-loop check above
confirmed that state communication still worked.

## Full-body teleoperation: compatibility pending

The controller-only Quest adapter is not the full-body SMPL/POSE pipeline.
The latter already exists in `gear_sonic/scripts/pico_manager_thread_server.py`
with `--input-source xrt` (XRoboToolkit) and `--input-source isaac-teleop`
(CloudXR). It requires a body skeleton, including lower-body joints, and a
separate teleop environment. Currently only `.venv_sim` is installed;
`isaacteleop`, `xrobotoolkit_sdk`, `smplx`, and `pyvista` are absent there.

Current NVIDIA documentation describes limited Quest 3/3S WebXR body support
through CloudXR; Quest Pro support for that browser path is not confirmed.
The native Meta Quest Pro Body Tracking API infers body pose, which by itself
is not proof of a usable lower-body stream. The documented PICO/XRoboToolkit
path in this checkout uses a headset, two controllers, and two ankle trackers.
The newer CloudXR PICO path has different enterprise/browser requirements.
Do not treat these hardware paths as interchangeable.

First restart the Quest bridge and open `https://<PC-IP>:8013/body-check` in
the headset. This diagnostic requests `body-tracking` as a required WebXR
feature and displays whether the browser grants it; it never publishes robot
commands. A successful feature request does not validate skeleton coverage,
tracking quality, or robot retargeting. A failure can also be a permission issue.
Full-body operation is not enabled yet.

References (checked 2026-09-28):
- [NVIDIA body tracking and limited Quest support](https://nvidia.github.io/IsaacTeleop/main/device/body_tracking.html)
- [Meta native body-tracking API](https://developers.meta.com/horizon/documentation/native/android/move-body-tracking/)
- [This checkout's full-body setup](docs/source/tutorials/vr_wholebody_teleop.md)
- [This checkout's Isaac Teleop setup](docs/source/tutorials/isaac_teleop_publisher_setup.md)

### Quest Pro feature grant confirmed; skeleton validation pending

The headset user confirmed that WebXR grants `body-tracking`. The updated
`/body-check` now samples `XRFrame.body` joint poses with `getPose` for 15 seconds
and displays joint availability, estimated-position flags, and movement ranges
relative to the hips. It runs entirely in the headset browser: no pose upload,
server-side report, or robot commands. Reload the existing page to use it;
no bridge restart is required for the updated static page/script.

The WebXR draft permits emulated joints, so a populated skeleton alone is not
proof of independent leg tracking. Check lower-body availability and how it
changes when moving one foot while keeping the controllers still, then making
a small crouch. Full-body control remains disabled pending these observations.
Spec: https://immersive-web.github.io/body-tracking/
