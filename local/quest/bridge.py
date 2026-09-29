"""Experimental Quest WebXR -> SONIC VR_3PT adapter, loopback simulation only."""
import argparse
import asyncio
import contextlib
import json
import msgpack
from pathlib import Path
import ssl
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as R
from aiohttp import web
import zmq

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import get_g1_key_frame_poses
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message, build_planner_message

# WebXR: +X right, +Y up, -Z forward -> SONIC: +X forward, +Y left, +Z up.
BASIS = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
TIMEOUT = 0.35


def validate_frame(data):
    if data.get('type') != 'frame' or data.get('tracked') is not True:
        raise ValueError('Head and BOTH selected inputs must be tracked')
    seq = data['seq']
    if not isinstance(seq, int) or seq < 0:
        raise ValueError('Invalid sequence')
    poses = []
    for key in ('left', 'right', 'head'):
        pose = np.asarray(data[key], dtype=float)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError('Invalid pose')
        if np.max(np.abs(pose[:3])) > 20 or not .95 < np.linalg.norm(pose[3:]) < 1.05:
            raise ValueError('Invalid pose range/quaternion')
        poses.append(pose)
    pinch = np.asarray(data.get('pinch', [0, 0]), dtype=float)
    if pinch.shape != (2,) or not np.isfinite(pinch).all():
        raise ValueError('Invalid pinch')
    return seq, np.stack(poses), np.clip(pinch, 0, 1)


class Mapping:
    def __init__(self, poses, robot_poses):
        # Calibrate yaw once; preserve wrist rotation relative to calibration.
        head_rot = R.from_quat(poses[2, 3:]).as_matrix()
        forward = BASIS @ head_rot @ np.array([0., 0., -1.])
        yaw = np.arctan2(forward[1], forward[0])
        self.axes = R.from_euler('z', -yaw).as_matrix() @ BASIS
        self.origin = poses.copy()
        self.ref_pos = np.array([robot_poses[k]['position'] for k in ('left_wrist', 'right_wrist', 'torso')])
        self.ref_rot = [R.from_quat(robot_poses[k]['orientation_xyzw']) for k in ('left_wrist', 'right_wrist', 'torso')]
        self.start_rot = [R.from_matrix(self.axes @ R.from_quat(p[3:]).as_matrix() @ BASIS.T) for p in poses]

    def apply(self, poses):
        # Head-relative wrists avoid translating the robot arms when the user steps.
        delta = (poses[:, :3] - poses[2, :3]) - (self.origin[:, :3] - self.origin[2, :3])
        delta = delta @ self.axes.T
        if np.max(np.linalg.norm(delta[:2], axis=1)) > .65:
            raise ValueError('Hand moved beyond calibrated reach; stop and recalibrate')
        pos = self.ref_pos + delta
        rotations = []
        for i, p in enumerate(poses):
            live = R.from_matrix(self.axes @ R.from_quat(p[3:]).as_matrix() @ BASIS.T)
            rotations.append(live * self.start_rot[i].inv() * self.ref_rot[i])
        # Looking around is not a measurement of torso motion. Keep the
        # calibrated torso target fixed for this head/wrist-only adapter.
        rotations[2] = self.ref_rot[2]
        pos[2] = self.ref_pos[2].copy()
        quat = np.array([r.as_quat(scalar_first=True) for r in rotations])
        return pos, quat


class TargetFilter:
    """Smooth targets in robot coordinates; cap motion even after loop stalls."""
    def __init__(self, pos, quat):
        self.pos = pos.copy()
        self.rot = R.from_quat(quat[:, [1, 2, 3, 0]])
        self.pinch = np.zeros(2)
        self.at = time.monotonic()

    def apply(self, pos, quat, pinch):
        now = time.monotonic()
        dt = float(np.clip(now - self.at, 0., .04))
        self.at = now
        alpha = -np.expm1(-dt / .12)
        step = (pos - self.pos) * alpha
        distance = np.linalg.norm(step, axis=1, keepdims=True)
        self.pos += step * np.minimum(1., .4 * dt / np.maximum(distance, 1e-12))
        desired = R.from_quat(quat[:, [1, 2, 3, 0]])
        turn = (desired * self.rot.inv()).as_rotvec() * alpha
        angle = np.linalg.norm(turn, axis=1, keepdims=True)
        turn *= np.minimum(1., 1.5 * dt / np.maximum(angle, 1e-12))
        self.rot = R.from_rotvec(turn) * self.rot
        self.pinch += np.clip((pinch - self.pinch) * alpha, -2. * dt, 2. * dt)
        return self.pos.copy(), self.rot.as_quat(scalar_first=True), self.pinch.copy()


class ControllerWalking:
    """Body-relative thumbsticks with bounded translation and yaw rates."""
    def __init__(self, heading=0.):
        self.heading = heading
        self.velocity = np.zeros(2)
        self.turning = False
        self.at = time.monotonic()

    @staticmethod
    def deadzone(value):
        return np.sign(value) * max(0., (abs(value) - .2) / .8)

    def update(self, axes):
        now = time.monotonic()
        dt = float(np.clip(now - self.at, 0., .04))
        self.at = now
        target = np.array([self.deadzone(x) for x in axes[:2]])
        target *= .35 / max(1., np.linalg.norm(target))
        delta = target - self.velocity
        self.velocity += delta * min(1., .5 * dt / max(np.linalg.norm(delta), 1e-12))
        turn = self.deadzone(axes[2])
        self.turning = abs(turn) > 0
        self.heading = (self.heading + .6 * turn * dt + np.pi) % (2 * np.pi) - np.pi

    def command(self):
        speed = float(np.linalg.norm(self.velocity))
        if speed < .01:
            return [0., 0., 0.], 0.
        x, y = self.velocity / speed
        c, s = np.cos(self.heading), np.sin(self.heading)
        return [float(c*x-s*y), float(s*x+c*y), 0.], speed


class Bridge:
    def __init__(self, pub, robot):
        self.pub, self.robot = pub, robot
        self.ws = None
        self.latest = None
        self.last_seq = -1
        self.last_rx = 0.
        self.mapping = None
        self.target_filter = None
        self.recovery_target = None
        self.recovery_filter = None
        self.active = False
        self.standing = False
        self.feedback_since = None
        self.neutral_since = None
        self.stop_until = 0.
        self.reason = 'Waiting for Quest'
        self.crouch = False
        self.height = .78
        self.lowstate = None
        self.lowstate_at = 0.
        self.count = 0
        self.started_at = 0.
        self.feedback_at = 0.
        self.tracking_detail = 'No XR frames yet: enter passthrough in Quest Browser'
        self.input_mode = None
        self.locomotion = 'off'
        self.walker = None
        self.heading = 0.
        self.drive = np.zeros(3)
        self.last_diagnostic = 0.
        self.start_after = None
        self.start_deadline = None
        self.tracked_since = None

    def on_state(self, state):
        self.lowstate = np.array([m.q for m in state.motor_state[:29]], dtype=float)
        self.lowstate_at = time.monotonic()

    def fresh(self):
        return self.latest is not None and time.monotonic() - self.last_rx < TIMEOUT

    def stop(self, reason):
        was_active = self.active or self.standing
        self.standing = False
        self.start_after = None
        self.start_deadline = None
        self.active = False
        self.mapping = None
        self.target_filter = None
        self.walker = None
        self.recovery_target = self.recovery_filter = None
        self.neutral_since = None
        if was_active:
            self.stop_until = time.monotonic() + 1.
        self.reason = reason + ('; controller stopped: restart controller and bridge before recalibrating' if was_active else '')
        if was_active:
            print('STOP:', reason, flush=True)

    def prepare_recovery(self):
        # G1's URDF zero elbow angle already has the forearm pointing forward
        # (roughly 90 degrees to the upper arm). Do not set elbow q to pi/2.
        if self.lowstate is None or time.monotonic() - self.lowstate_at >= TIMEOUT:
            raise ValueError('Fresh simulation state required for arm recovery')
        q = self.robot.get_configuration_from_actuated_joints(body_actuated_joint_values=np.array(self.lowstate))
        current = get_g1_key_frame_poses(self.robot, q=q)
        ready = get_g1_key_frame_poses(self.robot)
        # Preserve the torso reference; only the arms move to the ready pose.
        ready['torso'] = current['torso']
        def targets(refs):
            keys = ('left_wrist', 'right_wrist', 'torso')
            return (np.array([refs[k]['position'] for k in keys]),
                    np.array([R.from_quat(refs[k]['orientation_xyzw']).as_quat(scalar_first=True) for k in keys]))
        self.recovery_target = targets(ready)
        self.recovery_filter = TargetFilter(*targets(current))

    def idle(self):
        self.height += float(np.clip(.78 - self.height, -.003, .003))
        extra = {}
        settled = True
        if self.recovery_target is not None:
            # Locally generated ready targets, independent of lost XR input.
            pos, quat, _ = self.recovery_filter.apply(*self.recovery_target, np.zeros(2))
            extra = dict(vr_3pt_position=pos.ravel(), vr_3pt_orientation=quat.ravel())
            goal_pos, goal_quat = self.recovery_target
            angle = (R.from_quat(quat[:, [1, 2, 3, 0]]) *
                     R.from_quat(goal_quat[:, [1, 2, 3, 0]]).inv()).magnitude()
            settled = np.max(np.linalg.norm(pos - goal_pos, axis=1)) < .01 and np.max(angle) < .05
        if self.height >= .779 and settled and self.neutral_since is None:
            self.neutral_since = time.monotonic()
        self.pub.send(build_planner_message(
            mode=4 if self.height < .73 else 0, movement=[0, 0, 0],
            facing=[float(np.cos(self.heading)), float(np.sin(self.heading)), 0.], speed=0., height=self.height,
            left_hand_position=np.zeros(7), right_hand_position=np.zeros(7), **extra))

    def pause(self, reason):
        was_active = self.active
        self.standing = self.standing or self.active
        self.active = False
        self.mapping = self.target_filter = self.walker = None
        self.drive = np.zeros(3)
        self.crouch = False
        self.neutral_since = None
        self.start_after = self.start_deadline = None
        if was_active:
            try:
                self.prepare_recovery()
            except (ValueError, RuntimeError) as e:
                self.stop('Arm recovery unavailable: ' + str(e))
                return
        pose_description = 'standing with bent-elbow ready arms' if self.recovery_target is not None else 'neutral standing'
        self.reason = reason + (f'; teleop paused, returning to {pose_description}. Release then hold both thumbstick clicks to recalibrate/resume (or type go).'
                                if self.standing else '; teleop inactive.')
        if self.standing:
            self.idle()
            print(self.reason, flush=True)

    def start_standing(self):
        if time.monotonic() < self.stop_until:
            raise ValueError('Emergency stop in progress; restart controller and bridge')
        if self.lowstate is None or time.monotonic() - self.lowstate_at >= TIMEOUT:
            raise ValueError('Fresh simulation state required')
        if not self.standing and not self.active:
            self.standing = True
            self.started_at = time.monotonic()
            self.feedback_since = None
            self.idle()
            self.pub.send(build_command_message(True, False, True))

    def on_feedback(self):
        now = time.monotonic()
        if now - self.feedback_at >= 1. or self.feedback_since is None:
            self.feedback_since = now
        self.feedback_at = now

    def standing_ready(self):
        now = time.monotonic()
        return (self.standing and self.neutral_since is not None
                and self.height >= .779 and now - self.neutral_since >= 2.
                and self.feedback_since is not None
                and now - self.feedback_since >= 2. and now - self.feedback_at < 1.)

    def receive(self, data):
        seq, poses, pinch = validate_frame(data)
        mode = data.get('input_mode', 'hands')
        if mode not in ('hands', 'controllers'):
            raise ValueError('Invalid input mode')
        locomotion = data.get('locomotion', 'off')
        if locomotion not in ('off', 'sticks'):
            raise ValueError('Use controllers with thumbsticks; reload the updated headset page')
        drive = np.asarray(data.get('drive', [0, 0, 0]), dtype=float)
        if drive.shape != (3,) or not np.isfinite(drive).all() or np.max(np.abs(drive)) > 1:
            raise ValueError('Invalid thumbstick axes')
        if locomotion == 'sticks' and mode != 'controllers':
            raise ValueError('Thumbstick walking requires controllers')
        if seq <= self.last_seq:
            raise ValueError('Repeated or out-of-order tracking frame')
        if self.mapping is not None and not self.fresh():
            self.pause('Tracking gap')
        if self.input_mode is not None and (mode != self.input_mode or locomotion != self.locomotion):
            self.pause('Input mode changed; recalibration required')
            self.tracked_since = None
        self.input_mode = mode
        self.locomotion = locomotion
        self.drive = drive
        if self.active and self.walker is not None:
            self.walker.update(drive)
            self.heading = self.walker.heading
        if not self.fresh() or self.tracked_since is None:
            self.tracked_since = time.monotonic()
        self.last_seq = seq
        self.latest = (poses, pinch)
        self.last_rx = time.monotonic()
        self.count += 1
        self.tracking_detail = f'Head and both {self.input_mode} tracked'

    def command(self, command):
        command = command.strip().lower()
        if command == 'toggle':
            command = 'p' if self.active or self.start_after is not None else 'go'
        if command == 'go':
            if self.active:
                raise ValueError('Already active; use p to pause or o for emergency stop')
            self.start_standing()
            self.mapping = None
            self.tracked_since = None
            self.start_after = time.monotonic() + 5.
            self.start_deadline = time.monotonic() + 20.
            self.reason = 'STANDING STARTED: suspension releases automatically. Keep both selected inputs tracked; teleop starts after at least 5 seconds, fresh controller feedback and continuous tracking. p cancels teleop; o emergency stops.'
        elif command == 'p':
            self.pause('Paused by operator')
        elif command == 'o':
            self.stop('Stopped by operator')
        elif command == 'c':
            self.start_after = self.start_deadline = None
            if self.active:
                raise ValueError('Stop before recalibrating')
            if not self.standing_ready():
                raise ValueError('Start with go; wait for standing control before calibration')
            if self.locomotion == 'sticks' and np.max(np.abs(self.drive)) > .2:
                raise ValueError('Center the thumbsticks before calibration')
            if not self.fresh():
                raise ValueError('No fresh head + hand tracking. ' + self.tracking_detail)
            if self.lowstate is None or time.monotonic() - self.lowstate_at > TIMEOUT:
                raise ValueError('No fresh simulated robot state; start MuJoCo and controller first')
            q = self.robot.get_configuration_from_actuated_joints(body_actuated_joint_values=self.lowstate.copy())
            refs = get_g1_key_frame_poses(self.robot, q=q)
            self.mapping = Mapping(self.latest[0], refs)
            self.height = .78
            self.reason = 'Calibrated. Type r to start'
        elif command == 'r':
            self.start_after = self.start_deadline = None
            if not self.standing_ready():
                raise ValueError('Standing controller is not ready; use go')
            if not self.fresh() or self.mapping is None:
                raise ValueError('Fresh tracking and calibration required: c then r')
            if time.monotonic() - self.lowstate_at > TIMEOUT:
                raise ValueError('Simulation state is stale')
            self.target_filter = TargetFilter(*self.mapping.apply(self.mapping.origin))
            self.walker = ControllerWalking(self.heading) if self.locomotion == 'sticks' else None
            self.active = True
            self.recovery_target = self.recovery_filter = None
            self.neutral_since = None
            self.started_at = time.monotonic()
            self.stop_until = 0.
            self.reason = f'ACTIVE: {self.input_mode}; locomotion={self.locomotion}'
        elif command == 'h':
            if self.locomotion == 'sticks':
                raise ValueError('Head crouch disabled in controller-only mode')
            self.crouch = not self.crouch
            print('Head-height crouch:', self.crouch, '(experimental)', flush=True)
        elif command == 's':
            pass
        else:
            raise ValueError('Commands: go delayed calibrate + start, c calibrate, r start, p pause, o emergency stop, h crouch toggle, s status, q quit')
        print(self.status(), flush=True)

    def status(self):
        return dict(status=self.reason, active=self.active, standing=self.standing, start_pending=self.start_after is not None, calibrated=self.mapping is not None,
                    input_mode=self.input_mode, locomotion=self.locomotion, tracking=self.fresh(), tracking_detail=self.tracking_detail, frames=self.count, head_height_crouch=self.crouch,
                    controller_feedback=time.monotonic() - self.feedback_at < 1.)

    def tick(self):
        now = time.monotonic()
        if (self.active or self.standing) and now - self.lowstate_at > TIMEOUT:
            self.stop('Simulation state lost')
        if (self.active or self.standing) and now - self.started_at > 5. and now - self.feedback_at > 1.:
            self.stop('Controller feedback lost')
        if self.start_after is not None:
            if not self.fresh():
                self.tracked_since = None
            if now >= self.start_deadline:
                self.pause('Start cancelled: tracking or standing controller not ready')
                print(self.reason, flush=True)
            elif (now >= self.start_after and self.fresh()
                  and self.tracked_since is not None and now - self.tracked_since >= 1.
                  and self.lowstate is not None and now - self.lowstate_at < TIMEOUT
                  and self.standing_ready()
                  and (self.locomotion != 'sticks' or np.max(np.abs(self.drive)) <= .2)):
                # One explicitly requested start, never an automatic resume after loss.
                self.start_after = self.start_deadline = None
                try:
                    self.command('c')
                    self.command('r')
                except ValueError as e:
                    self.stop('Start cancelled: ' + str(e))
                    print(self.reason, flush=True)
        if self.mapping is not None and not self.fresh():
            self.pause('Tracking lost')
        if self.active:
            poses, pinch = self.latest
            try:
                pos, quat = self.mapping.apply(poses)
            except ValueError as e:
                self.stop(str(e))
            else:
                if self.target_filter is None:
                    self.target_filter = TargetFilter(*self.mapping.apply(self.mapping.origin))
                pos, quat, pinch = self.target_filter.apply(pos, quat, pinch)
                target = .78
                if self.crouch:
                    drop = self.mapping.origin[2, 1] - poses[2, 1]
                    target = float(np.clip(.78 - .6 * max(0., drop), .55, .78))
                self.height += float(np.clip(target - self.height, -.003, .003))
                # Conservative pinch synergy adapted from upstream index-close pose.
                left = pinch[0] * np.array([-.5, .7, .7, -1.5, -1.5, -.6, -1.5])
                right = pinch[1] * np.array([-.5, -.7, -.7, 1.5, 1.5, .6, 1.5])
                movement, speed = self.walker.command() if self.walker else ([0., 0., 0.], 0.)
                if self.height < .73:
                    movement, speed = [0., 0., 0.], 0.
                self.pub.send(build_planner_message(
                    mode=4 if self.height < .73 else (1 if speed > 0 or (self.walker and self.walker.turning) else 0),
                    movement=movement, facing=[float(np.cos(self.heading)), float(np.sin(self.heading)), 0.],
                    speed=speed, height=self.height,
                    left_hand_position=left, right_hand_position=right,
                    vr_3pt_position=pos.ravel(), vr_3pt_orientation=quat.ravel()))
        if self.standing and not self.active:
            self.idle()
        if now < self.stop_until:
            self.pub.send(build_command_message(False, True, True))

    async def websocket(self, request):
        if request.headers.get('Origin') != f'https://{request.host}':
            raise web.HTTPForbidden(text='Same-origin connection required')
        if self.ws is not None:
            raise web.HTTPConflict(text='One headset at a time')
        ws = web.WebSocketResponse(max_msg_size=8192, heartbeat=2)
        await ws.prepare(request)
        self.ws = ws
        self.last_seq = -1
        await ws.send_json(self.status())
        print('Quest browser connected; enter passthrough and track both selected inputs.', flush=True)
        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        if data.get('type') == 'command':
                            if data.get('command') not in ('go', 'p', 'o', 'toggle'):
                                raise ValueError('Unsupported browser command')
                            try:
                                self.command(data['command'])
                            except ValueError as e:
                                self.reason = str(e)
                            await ws.send_json(self.status())
                            continue
                        detail = data.get('diagnostics', {})
                        self.tracking_detail = str(detail)[:400] if detail else 'No tracking details (reload the Quest page)'
                        self.receive(data)
                    except (ValueError, KeyError, TypeError) as e:
                        self.latest = None
                        self.tracked_since = None
                        if time.monotonic() - self.last_diagnostic > 3.:
                            print('Tracking unavailable:', self.tracking_detail, flush=True)
                            self.last_diagnostic = time.monotonic()
                        if self.mapping is not None:
                            self.pause(str(e))
                    await ws.send_json(self.status())
        finally:
            self.pause('Quest disconnected')
            self.latest = None
            self.ws = None
        return ws


async def serve(args):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.setsockopt(zmq.SNDHWM, 3)
    pub.bind('tcp://127.0.0.1:5556')
    bridge = Bridge(pub, instantiate_g1_robot_model())
    feedback = context.socket(zmq.SUB)
    feedback.setsockopt(zmq.LINGER, 0)
    feedback.setsockopt(zmq.CONFLATE, 1)
    feedback.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
    feedback.connect('tcp://127.0.0.1:5557')
    ChannelFactoryInitialize(0, 'lo')
    subscriber = ChannelSubscriber('rt/lowstate', LowState_)
    subscriber.Init(bridge.on_state, 1)
    app = web.Application()
    app.router.add_get('/', lambda r: web.FileResponse(Path(__file__).with_name('index.html')))
    app.router.add_get('/body-check', lambda r: web.FileResponse(Path(__file__).with_name('body-check.html')))
    app.router.add_get('/body-check.js', lambda r: web.FileResponse(Path(__file__).with_name('body-check.js')))
    app.router.add_get('/input.js', lambda r: web.FileResponse(Path(__file__).with_name('input.js')))
    app.router.add_get('/quest.js', lambda r: web.FileResponse(Path(__file__).with_name('quest.js')))
    app.router.add_get('/ws', bridge.websocket)
    app.router.add_get('/status', lambda r: web.json_response(bridge.status()))
    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(args.cert, args.key)
    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, args.bind, args.port, ssl_context=ssl_ctx).start()
    print(f'Quest HTTPS ready on port {args.port}. Open https://<PC-WiFi-IP>:{args.port}', flush=True)
    print('SIMULATION ONLY. go + Enter: calibrate + start after 5s with both selected inputs tracked; p pauses; o emergency stops; c/r manual; h crouch; s status; q quit.', flush=True)
    loop = asyncio.get_running_loop()
    done = asyncio.Event()
    def terminal():
        line = sys.stdin.readline()
        if not line:
            loop.remove_reader(sys.stdin)
        elif line.strip().lower() == 'q':
            done.set()
        else:
            try:
                bridge.command(line)
            except ValueError as e:
                print('Not started:', e, flush=True)
    if sys.stdin.isatty():
        loop.add_reader(sys.stdin, terminal)
    try:
        while not done.is_set():
            if feedback.poll(0):
                data = msgpack.unpackb(feedback.recv()[len(b'g1_debug'):], raw=False)
                if 'body_q_measured' in data:
                    bridge.on_feedback()
            bridge.tick()
            await asyncio.sleep(.02)
    finally:
        bridge.stop('Bridge closing')
        for _ in range(5):
            bridge.tick()
            await asyncio.sleep(.02)
        if sys.stdin.isatty():
            loop.remove_reader(sys.stdin)
        await runner.cleanup()
        subscriber.Close()
        pub.close()
        feedback.close()
        context.term()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8013)
    parser.add_argument('--cert', required=True)
    parser.add_argument('--key', required=True)
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(serve(parser.parse_args()))
