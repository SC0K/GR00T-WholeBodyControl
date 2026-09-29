"""Protocol, coordinate conversion, validation and stop-latch regression checks."""
import json
import time
import unittest
from unittest.mock import patch, Mock
import numpy as np
from scipy.spatial.transform import Rotation as R
from bridge import BASIS, Bridge, Mapping, TargetFilter, ControllerWalking, TIMEOUT, validate_frame, instantiate_g1_robot_model
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import HEADER_SIZE


def frame(seq=1):
    return dict(type='frame', tracked=True, seq=seq,
                left=[-.25,1.2,-.3,0,0,0,1], right=[.25,1.2,-.3,0,0,0,1],
                head=[0,1.6,0,0,0,0,1], pinch=[0,1])

def refs():
    return {k:dict(position=np.array(p),orientation_xyzw=np.array([0,0,0,1])) for k,p in
            zip(('left_wrist','right_wrist','torso'),([.3,.2,.15],[.3,-.2,.15],[0,0,.4]))}

class Publisher:
    def __init__(self): self.messages=[]
    def send(self, message): self.messages.append(message)

class AdapterTests(unittest.TestCase):
    def test_calibration_axes_and_quaternions(self):
        _, poses, _ = validate_frame(frame())
        mapping=Mapping(poses,refs())
        p,q=mapping.apply(poses)
        np.testing.assert_allclose(p, mapping.ref_pos)
        np.testing.assert_allclose(q, [[1,0,0,0]]*3)
        moved=poses.copy(); moved[0,2]-=.1
        p,_=mapping.apply(moved)
        np.testing.assert_allclose(p[0]-mapping.ref_pos[0],[.1,0,0],atol=1e-8)
        # Room translation does not move pelvis-relative targets.
        moved=poses.copy(); moved[:,:3]+=[.2,0,.1]
        p,_=mapping.apply(moved)
        np.testing.assert_allclose(p,mapping.ref_pos,atol=1e-8)
        self.assertAlmostEqual(np.linalg.det(BASIS),1.)

    def test_yaw_calibration(self):
        _, poses, _=validate_frame(frame())
        rot=R.from_euler('y',70,degrees=True)
        poses[:,:3]=rot.apply(poses[:,:3]); poses[:,3:]=rot.as_quat()
        mapping=Mapping(poses,refs())
        moved=poses.copy(); moved[0,:3]+=rot.apply([0,0,-.1])
        p,q=mapping.apply(moved)
        np.testing.assert_allclose(p[0]-mapping.ref_pos[0],[.1,0,0],atol=1e-8)
        np.testing.assert_allclose(q,[[1,0,0,0]]*3,atol=1e-8)

    def test_invalid_tracking(self):
        for change in ({'tracked':False},{'head':[float('nan')]*7},{'left':[0]*7},{'pinch':[float('nan'),0]}):
            data=frame(); data.update(change)
            with self.assertRaises(ValueError): validate_frame(data)

    def test_wire_and_stale_latch(self):
        pub=Publisher(); bridge=Bridge(pub,instantiate_g1_robot_model())
        bridge.receive(frame()); bridge.mapping=Mapping(bridge.latest[0],refs())
        bridge.lowstate=np.zeros(29); bridge.lowstate_at=time.monotonic(); bridge.started_at=time.monotonic(); bridge.active=True
        bridge.tick()
        message=pub.messages[-1]
        self.assertTrue(message.startswith(b'planner'))
        header=json.loads(message[7:7+HEADER_SIZE].rstrip(b'\0'))
        fields={f['name']:f['shape'] for f in header['fields']}
        self.assertEqual(fields['vr_position'],[9]); self.assertEqual(fields['vr_orientation'],[12])
        self.assertEqual(fields['left_hand_joints'],[7])
        bridge.last_rx-=TIMEOUT+1; bridge.tick()
        self.assertFalse(bridge.active); self.assertIsNone(bridge.mapping)
        self.assertIsNone(bridge.target_filter)
        self.assertTrue(bridge.standing)
        self.assertIn('teleop paused', bridge.reason)
        self.assertTrue(pub.messages[-1].startswith(b'planner'))
        header=json.loads(pub.messages[-1][7:7+HEADER_SIZE].rstrip(b'\0'))
        self.assertIn('vr_position', [f['name'] for f in header['fields']])
        bridge.receive(frame(2)); bridge.tick()
        self.assertFalse(bridge.active)
        with self.assertRaises(ValueError): bridge.command('r')
        with self.assertRaises(ValueError): bridge.receive(frame(2))

    def test_no_output_before_start_and_controller_loss(self):
        pub=Publisher(); bridge=Bridge(pub,instantiate_g1_robot_model())
        bridge.receive(frame()); bridge.tick()
        self.assertEqual(pub.messages,[])
        bridge.mapping=Mapping(bridge.latest[0],refs())
        bridge.lowstate=np.zeros(29); bridge.lowstate_at=time.monotonic(); bridge.started_at=time.monotonic()-10
        bridge.active=True; bridge.tick()
        self.assertFalse(bridge.active)
        self.assertIn('Controller feedback lost',bridge.reason)

    def test_lost_simulation_stops(self):
        pub=Publisher(); bridge=Bridge(pub,instantiate_g1_robot_model())
        bridge.receive(frame()); bridge.mapping=Mapping(bridge.latest[0],refs()); bridge.active=True
        bridge.tick()
        self.assertFalse(bridge.active); self.assertIsNone(bridge.mapping)

class StabilityTests(unittest.TestCase):
    def test_looking_around_keeps_calibrated_torso(self):
        _, poses, _ = validate_frame(frame())
        reference = refs()
        reference['torso']['position'] = np.array([.02, -.01, .39])
        mapping = Mapping(poses, reference)
        poses[2, 3:] = R.from_euler('xyz', [40, 70, 30], degrees=True).as_quat()
        p, q = mapping.apply(poses)
        np.testing.assert_allclose(p[2], reference['torso']['position'])
        np.testing.assert_allclose(q[2], [1, 0, 0, 0], atol=1e-12)

    def test_position_rotation_pinch_rates_and_stall(self):
        p = np.zeros((3, 3))
        q = np.tile([1., 0., 0., 0.], (3, 1))
        desired = R.from_euler('z', [170., 170., 170.], degrees=True).as_quat(scalar_first=True)
        with patch('bridge.time.monotonic', return_value=100.) as clock:
            filt = TargetFilter(p, q)
            for t, dt in [(100.02, .02), (105., .04)]:
                old_p, old_rot, old_pinch = filt.pos.copy(), filt.rot, filt.pinch.copy()
                clock.return_value = t
                out_p, out_q, out_pinch = filt.apply(np.ones((3, 3)), desired, np.ones(2))
                self.assertLessEqual(np.max(np.linalg.norm(out_p-old_p, axis=1)), .4*dt+1e-12)
                out_rot = R.from_quat(out_q[:, [1, 2, 3, 0]])
                self.assertLessEqual(np.max((out_rot*old_rot.inv()).magnitude()), 1.5*dt+1e-12)
                self.assertLessEqual(np.max(out_pinch-old_pinch), 2*dt+1e-12)

    def test_quaternion_sign_flip_does_not_move_target(self):
        p = np.zeros((3, 3))
        q = np.tile([1., 0., 0., 0.], (3, 1))
        with patch('bridge.time.monotonic', return_value=100.) as clock:
            filt = TargetFilter(p, q)
            clock.return_value = 100.02
            out_p, out_q, _ = filt.apply(p, -q, np.zeros(2))
        np.testing.assert_allclose(out_p, p)
        np.testing.assert_allclose(out_q, q, atol=1e-12)


class DelayedStartTests(unittest.TestCase):
    def setUp(self):
        self.now=100.
        self.clock=patch('bridge.time.monotonic', side_effect=lambda:self.now)
        self.clock.start(); self.addCleanup(self.clock.stop)
        self.fk=patch('bridge.get_g1_key_frame_poses', return_value=refs())
        self.fk.start(); self.addCleanup(self.fk.stop)
        self.pub=Publisher(); self.b=Bridge(self.pub,Mock())
        self.b.lowstate=np.zeros(29); self.b.lowstate_at=self.now
        self.seq=0

    def tracked(self):
        self.seq+=1
        self.b.receive(frame(self.seq))
        self.b.lowstate_at=self.now
        self.b.on_feedback()

    def advance(self, start, end, tracking=True):
        for t in np.arange(start, end+.01, .1):
            self.now=float(t)
            self.b.lowstate_at=self.now
            self.b.on_feedback()
            if tracking: self.tracked()
            self.b.tick()

    def assert_no_vr(self):
        for m in self.pub.messages:
            if m.startswith(b'planner'):
                h=json.loads(m[7:7+HEADER_SIZE].rstrip(b'\0'))
                self.assertNotIn('vr_position', [f['name'] for f in h['fields']])
            else:
                self.assertNotEqual(m[-3:], bytes([0,1,1]))

    def test_go_stands_before_teleop_and_loss_pauses(self):
        self.b.command('go')
        self.assertTrue(self.b.standing); self.assertFalse(self.b.active)
        self.advance(100.,104.9)
        self.assert_no_vr()
        self.advance(105.,105.2)
        self.assertTrue(self.b.active)
        starts=[m for m in self.pub.messages if m.startswith(b'command')]
        self.assertEqual(len(starts),1)
        self.assertEqual(starts[0][-3:],bytes([1,0,1]))
        self.advance(105.3,106.,tracking=False)
        self.assertFalse(self.b.active); self.assertTrue(self.b.standing)
        self.assertIsNone(self.b.start_after)
        self.pub.messages.clear()
        self.advance(106.1,107.)
        self.assertFalse(self.b.active); self.assertIsNotNone(self.b.recovery_target)
        self.b.command('go'); self.advance(107.1,112.3)
        self.assertTrue(self.b.active)
        self.assertFalse(any(m.startswith(b'command') for m in self.pub.messages))

    def test_pause_cancels_start_and_deadline_keeps_standing(self):
        self.b.command('go'); self.b.command('p')
        self.assertIsNone(self.b.start_after)
        self.advance(100.,101.)
        self.assertFalse(self.b.active)
        self.b.command('go'); self.advance(101.1,122.,tracking=False)
        self.assertIsNone(self.b.start_after)
        self.assertFalse(self.b.active); self.assertTrue(self.b.standing)
        self.assert_no_vr()

    def test_toggle_and_centered_sticks_required(self):
        self.b.command('toggle')
        self.assertIsNotNone(self.b.start_after)
        for t in np.arange(100.,106.,.1):
            self.now=float(t); self.seq+=1
            data=frame(self.seq); data.update(input_mode='controllers',locomotion='sticks',drive=[1,0,0])
            self.b.receive(data); self.b.lowstate_at=self.now; self.b.on_feedback(); self.b.tick()
        self.assertFalse(self.b.active)
        for t in np.arange(106.,107.,.1):
            self.now=float(t); self.seq+=1
            data=frame(self.seq); data.update(input_mode='controllers',locomotion='sticks',drive=[0,0,0])
            self.b.receive(data); self.b.lowstate_at=self.now; self.b.on_feedback(); self.b.tick()
        self.assertTrue(self.b.active)
        self.b.command('toggle'); self.assertFalse(self.b.active)
        self.assertTrue(self.b.standing); self.assertFalse(self.b.standing_ready())

    def test_fresh_simulation_required(self):
        self.b.lowstate_at=90.
        with self.assertRaises(ValueError): self.b.command('go')
        self.assertEqual(self.pub.messages,[])

    def test_missing_controller_feedback_stops(self):
        self.b.command('go')
        self.now=106.; self.b.lowstate_at=self.now; self.b.tick()
        self.assertFalse(self.b.standing)
        self.assertEqual(self.pub.messages[-1][-3:],bytes([0,1,1]))

    def test_emergency_stop_while_paused_and_sim_loss(self):
        self.b.command('go'); self.b.command('p'); self.b.command('o'); self.b.tick()
        self.assertFalse(self.b.standing)
        self.assertEqual(self.pub.messages[-1][-3:],bytes([0,1,1]))
        with self.assertRaises(ValueError): self.b.command('go')

    def test_intermittent_tracking_does_not_arm(self):
        self.b.command('go')
        for t in np.arange(100., 106., .5):
            self.now=float(t); self.tracked(); self.b.tick()
        self.assertFalse(self.b.active); self.assert_no_vr()

    def test_lost_simulation_while_paused_stops(self):
        self.b.command('go'); self.b.command('p')
        self.now=101.; self.b.tick()
        self.assertFalse(self.b.standing)
        self.assertEqual(self.pub.messages[-1][-3:],bytes([0,1,1]))

class ControllerWalkingTests(unittest.TestCase):
    def test_rates_deadzone_and_body_relative_heading(self):
        with patch('bridge.time.monotonic', return_value=100.) as clock:
            walk=ControllerWalking(np.pi/2)
            walk.update([.1,-.1,.1]); self.assertEqual(walk.command()[1],0.)
            old=walk.velocity.copy()
            for i in range(1,101):
                clock.return_value=100+i*.02; walk.update([1,0,0])
                self.assertLessEqual(np.linalg.norm(walk.velocity-old), .01+1e-12)
                self.assertLessEqual(walk.command()[1],.35+1e-12)
                old=walk.velocity.copy()
            movement,speed=walk.command()
            np.testing.assert_allclose(movement,[0,1,0],atol=1e-12)
            self.assertAlmostEqual(speed,.35)
            clock.return_value=102.02; walk.update([0,0,-1])
            self.assertAlmostEqual(walk.heading,np.pi/2-.012)
            for i in range(102,202):
                clock.return_value=100+i*.02; walk.update([0,0,0])
            self.assertEqual(walk.command()[1],0.)

    def test_pause_clears_motion_and_crouch_preserves_heading(self):
        b=Bridge(Publisher(),instantiate_g1_robot_model()); b.standing=b.active=True; b.crouch=True
        b.lowstate=np.zeros(29); b.lowstate_at=time.monotonic()
        b.heading=.4; b.height=.6; b.walker=ControllerWalking(.4)
        b.mapping=Mock(); b.pause('Connection lost')
        self.assertFalse(b.active); self.assertFalse(b.crouch)
        self.assertIsNone(b.walker); self.assertIsNone(b.mapping)
        self.assertIsNone(b.neutral_since); self.assertFalse(b.standing_ready())
        with patch('bridge.time.monotonic', return_value=time.monotonic()) as clock:
            for _ in range(150):
                clock.return_value += .02
                b.idle()
        self.assertAlmostEqual(b.height,.78); self.assertAlmostEqual(b.heading,.4)
        self.assertIsNotNone(b.neutral_since)
        packet=b.pub.messages[-1]
        header=json.loads(packet[7:7+HEADER_SIZE].rstrip(b'\0'))
        self.assertIn('vr_position',[f['name'] for f in header['fields']])

    def test_old_head_follow_and_invalid_sticks_rejected(self):
        b=Bridge(Publisher(),instantiate_g1_robot_model())
        for axes in [[2,0,0],[float('nan'),0,0],[0,0]]:
            data=frame(); data.update(input_mode='controllers',locomotion='sticks',drive=axes)
            with self.assertRaises(ValueError): b.receive(data)
        data=frame(); data['locomotion']='head'
        with self.assertRaises(ValueError): b.receive(data)

    def test_input_mode_switch_requires_recalibration(self):
        b=Bridge(Publisher(),instantiate_g1_robot_model()); b.receive(frame())
        b.lowstate=np.zeros(29); b.lowstate_at=time.monotonic()
        b.mapping=Mapping(b.latest[0],refs()); b.active=True
        data=frame(2); data['input_mode']='controllers'; b.receive(data)
        self.assertFalse(b.active); self.assertIsNone(b.mapping); self.assertTrue(b.standing)

class ArmRecoveryTests(unittest.TestCase):
    def test_ready_geometry_and_rate_limited_recovery_without_xr(self):
        robot = instantiate_g1_robot_model()
        b = Bridge(Publisher(), robot)
        q = robot.get_body_actuated_joints(robot.default_body_pose).copy()
        # Simulate the low-arm standing posture before disconnect.
        names = robot.supplemental_info.body_actuated_joints
        for side in ('left', 'right'):
            q[names.index(side + '_elbow_joint')] = 1.0
        with patch('bridge.time.monotonic', return_value=100.) as clock:
            b.lowstate = q; b.lowstate_at = 100.
            b.active = b.standing = True
            b.pause('Controller lost')
            goal_pos, goal_quat = b.recovery_target
            self.assertGreater(np.min(goal_pos[:2, 0]), .25)
            # Check physical forearm/upper-arm directions, not joint q=pi/2.
            robot.cache_forward_kinematics(robot.default_body_pose, auto_clip=False)
            for side in ('left', 'right'):
                shoulder = robot.frame_placement(side + '_shoulder_roll_link').translation.copy()
                elbow = robot.frame_placement(side + '_elbow_link').translation.copy()
                wrist = robot.frame_placement(side + '_wrist_yaw_link').translation.copy()
                upper = elbow - shoulder; lower = wrist - elbow
                angle = np.degrees(np.arccos(np.dot(upper, lower) / np.linalg.norm(upper) / np.linalg.norm(lower)))
                self.assertTrue(80 < angle < 100, angle)
            self.assertIsNone(b.mapping); self.assertIsNone(b.walker)
            for _ in range(200):
                before = b.recovery_filter.pos.copy()
                clock.return_value += .02
                b.idle()
                self.assertLessEqual(np.max(np.linalg.norm(b.recovery_filter.pos - before, axis=1)), .008 + 1e-10)
            np.testing.assert_allclose(b.recovery_filter.pos, goal_pos, atol=.001)
            self.assertFalse(b.active)
            self.assertIsNotNone(b.neutral_since)
            # Repeated loss does not restart the trajectory or resume teleop.
            old_filter = b.recovery_filter
            b.pause('Still disconnected')
            self.assertIs(b.recovery_filter, old_filter)
            b.command('o')
            self.assertIsNone(b.recovery_target)
            self.assertFalse(b.standing)

class AutoReleaseTests(unittest.TestCase):
    def test_only_valid_feedback_releases_once(self):
        import msgpack
        from gear_sonic.utils.mujoco_sim.auto_release import AutoRelease
        with patch('gear_sonic.utils.mujoco_sim.auto_release.zmq.Context') as context:
            socket=context.instance.return_value.socket.return_value
            release=AutoRelease(); band=Mock(enable=True)
            socket.poll.return_value=0
            release.poll(band); self.assertTrue(band.enable)
            socket.poll.return_value=1
            for data in [b'bad', msgpack.packb({'body_q_measured':[0]})]:
                socket.recv.return_value=b'g1_debug'+data
                release.poll(band); self.assertTrue(band.enable)
            socket.recv.return_value=b'g1_debug'+msgpack.packb({'body_q_measured':[0]*29})
            release.poll(band); self.assertFalse(band.enable)
            band.enable=True
            release.poll(band); self.assertTrue(band.enable)
            socket.close.assert_called_once()

if __name__=='__main__': unittest.main()
