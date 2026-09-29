# OptiTrack body teleoperation preparation

`setup.json` records the intended setup. **It is not loaded by the existing
Quest bridge or SONIC controller. No NatNet receiver or OptiTrack-to-SMPL
converter is implemented yet, so this profile does not enable live mocap.**
The existing Quest controller/joystick workflow remains available.

## Selected setup

- Full-body skeleton from Motive: arms, torso and legs; no finger tracking.
- Open robot hands: explicitly send seven zero joint targets for each Dex3 hand
  on every output frame, using the same open-hand convention as the local Quest
  adapter. Do not merely omit hand fields and assume previous grasps are cleared.
- Neutral robot wrist joint values until the wrist orientation mapping is validated.
- Motive to Linux over NatNet unicast; use global skeleton coordinates and meters.
- Convert the skeleton to SONIC protocol v3, publish `pose` on loopback port 5556.
- Use `zmq_manager` so a future adapter can switch from full-body streaming to a
  validated standing fallback on tracking loss. The plain `zmq` receiver has
  different pause behavior and does not inherit the Quest fallback.
- Simulation first, explicit start and resume. Timeout/fallback settings in the
  profile are requirements for the future adapter, not an installed watchdog.

## Details needed from the Motive setup

Fill the null connection fields only after checking the actual Motive setup:

1. Motive version and matching NatNet SDK/protocol version.
2. Motive PC IPv4 address and this Linux PC's interface address on the same network.
3. Full-body skeleton name/ID and streaming bone naming convention.
4. A skeleton description (bone names, IDs and hierarchy), plus its reference-pose
   definition. These determine the mapping and orientation offsets.

In Motive, enable streaming of the full skeleton and select unicast, global
skeleton coordinates, and meters. Check the options against your Motive version.
Raw marker positions or a few rigid bodies are not a complete body skeleton.
Do not use the Quest hotspot address for the Motive PC unless it actually has
that address. Leave unknown values null rather than guessing.

## Adapter work still required

Receive and validate NatNet skeleton frames; select the configured skeleton;
convert coordinate axes, bone reference orientations and joint hierarchy;
produce SMPL local rotations and SONIC root-local joints; resample/buffer frames
for the deployed encoder; and publish with the existing
`gear_sonic.utils.teleop.zmq.zmq_planner_sender.pack_pose_message` builder.

Protocol v3 requires `smpl_pose`, `smpl_joints`, `joint_pos`, `joint_vel`,
`frame_index` and body orientation (the built-in publisher uses `body_quat_w`).
Include explicit `left_hand_joints` and `right_hand_joints` from this profile.
Use the actual upstream SMPL conventions, not raw Motive positions renamed SMPL.

Validate a still reference pose, individual limb motions, heading, scale,
tracking loss and explicit resume offline and in MuJoCo before live use.
Only one publisher can bind port 5556: stop the Quest bridge before running a
future mocap adapter.

## Optional Quest triggers

Open hands are selected for now. A future trigger-only receiver can supply grasp
values independently of the mocap body stream. It must not also publish Quest
arm/walking commands on port 5556. Trigger loss should open the hands while
valid mocap body tracking continues. This integration is not implemented;
changing the JSON alone cannot enable it.

## References

- https://docs.optitrack.com/motive/data-streaming
- https://www.optitrack.com/software/natnet-sdk
- ../../docs/source/tutorials/zmq.md
