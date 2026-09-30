import json
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from retarget import Mapping, Skeleton, Y_TO_Z
from bridge import Session


@pytest.fixture
def sample():
    layout = [
        ("Hips", 0, [0, 0.9, 0]),
        ("Spine", 1, [0, 0.1, 0]),
        ("Spine1", 2, [0, 0.2, 0]),
        ("Neck", 3, [0, 0.2, 0]),
        ("Head", 4, [0, 0.15, 0]),
        ("LeftShoulder", 3, [0.04, 0.14, 0]),
        ("LeftArm", 6, [0.12, 0, 0]),
        ("LeftForeArm", 7, [0.3, 0, 0]),
        ("LeftHand", 8, [0.2, 0, 0]),
        ("RightShoulder", 3, [-0.04, 0.14, 0]),
        ("RightArm", 10, [-0.12, 0, 0]),
        ("RightForeArm", 11, [-0.3, 0, 0]),
        ("RightHand", 12, [-0.2, 0, 0]),
        ("LeftUpLeg", 1, [0.09, 0, 0]),
        ("LeftLeg", 14, [0, -0.4, 0]),
        ("LeftFoot", 15, [0, -0.4, 0]),
        ("LeftToeBase", 16, [0, -0.06, 0.14]),
        ("RightUpLeg", 1, [-0.09, 0, 0]),
        ("RightLeg", 18, [0, -0.4, 0]),
        ("RightFoot", 19, [0, -0.4, 0]),
        ("RightToeBase", 20, [0, -0.06, 0.14]),
    ]
    description = [
        dict(id=i + 1, parent_id=parent, name="Test_" + name, offset=offset)
        for i, (name, parent, offset) in enumerate(layout)
    ]
    bones = {}
    for bone in description:
        parent = bones.get(bone["parent_id"])
        pos = Y_TO_Z.apply(bone["offset"])
        if parent:
            pos += parent["position"]
        bones[bone["id"]] = {"position": pos, "quaternion": [0, 0, 0, 1], "flags": 0}
    frame = {"frame": 1, "timestamp": 1.0, "received": time.monotonic(), "bones": bones}
    return Skeleton(description), frame


def test_tpose_maps_to_neutral_smpl(sample):
    skeleton, frame = sample
    mapping = Mapping(skeleton, skeleton.reference(frame))
    data = mapping.convert(frame)
    assert np.max(np.abs(data["smpl_pose"])) < 1e-6
    assert data["smpl_joints"].shape == (24, 3)
    assert np.isfinite(data["smpl_joints"]).all()
    quat = data["body_quat_w"][[1, 2, 3, 0]]
    assert (R.from_quat(quat) * R.from_euler("z", 90, degrees=True)).magnitude() < 1e-5
    assert np.all(data["joint_pos"] == 0)


def test_heading_does_not_change_local_pose(sample):
    skeleton, frame = sample
    mapping = Mapping(skeleton, skeleton.reference(frame))
    turn = R.from_euler("z", 65, degrees=True)
    for bone in frame["bones"].values():
        bone["position"] = turn.apply(bone["position"])
        bone["quaternion"] = turn.as_quat()
    frame["received"] = time.monotonic()
    data = mapping.convert(frame)
    assert np.max(np.abs(data["smpl_pose"])) < 1e-5
    quat = data["body_quat_w"][[1, 2, 3, 0]]
    assert (R.from_quat(quat) * R.from_euler("z", 25, degrees=True)).magnitude() < 1e-5


def test_reference_bone_axes_are_removed(sample):
    skeleton, frame = sample
    for i, bone in enumerate(frame["bones"].values()):
        bone["quaternion"] = R.from_euler("xyz", [i * 0.01, -0.2, 0.3]).as_quat()
    mapping = Mapping(skeleton, skeleton.reference(frame))
    assert np.max(np.abs(mapping.convert(frame)["smpl_pose"])) < 1e-5


@pytest.mark.parametrize("failure", ["stale", "missing_leg", "nan", "bad_length", "zero_quat"])
def test_bad_tracking_rejected(sample, failure):
    skeleton, frame = sample
    if failure == "stale":
        frame["received"] -= 1
    elif failure == "missing_leg":
        del frame["bones"][skeleton.ids["LeftFoot"]]
    elif failure == "nan":
        frame["bones"][1]["position"][0] = float("nan")
    elif failure == "bad_length":
        frame["bones"][skeleton.ids["LeftFoot"]]["position"] += 1
    else:
        frame["bones"][1]["quaternion"] = [0, 0, 0, 0]
    with pytest.raises(ValueError):
        skeleton.validate(frame)


def test_wrong_reference_pose_rejected(sample):
    skeleton, frame = sample
    frame["bones"][skeleton.ids["LeftHand"]]["position"][2] += 0.3
    with pytest.raises(ValueError):
        skeleton.reference(frame)


class Publisher:
    def __init__(self):
        self.messages = []

    def send(self, data):
        self.messages.append(data)


def test_tracking_loss_requires_recalibration_and_explicit_resume(sample):
    skeleton, frame = sample
    session = Session(Publisher())
    session.mapping = object()
    session.state = "pose"
    session.source_valid = True
    frame["received"] -= 1
    session.source(frame, skeleton, time.monotonic())
    assert session.state == "standing"
    assert session.mapping is None
    frame["received"] = time.monotonic()
    session.source(frame, skeleton, time.monotonic())
    assert session.state == "standing"
    assert session.mapping is None


def test_frozen_counter_rejected(sample):
    skeleton, frame = sample
    session = Session(Publisher())
    session.source(frame, skeleton, frame["received"])
    now = frame["received"] + 0.4
    frame["received"] = now
    session.source(frame, skeleton, now)
    assert not session.source_valid


def test_missing_robot_feedback_stops(sample):
    _, frame = sample
    now = time.monotonic()
    session = Session(Publisher())
    session.state = "standing"
    session.started = now - 10
    session.lowstate_at = now
    session.tick(frame, now)
    assert session.state == "idle"


def test_no_automatic_start(sample):
    skeleton, frame = sample
    publisher = Publisher()
    session = Session(publisher)
    session.source(frame, skeleton, time.monotonic())
    session.tick(frame, time.monotonic())
    assert session.state == "idle"
    assert publisher.messages == []


def test_leg_rotation_uses_calibrated_smpl_axes(sample):
    skeleton, frame = sample
    calibration = skeleton.reference(frame)
    mapping = Mapping(skeleton, calibration)
    delta = R.from_rotvec([0.2, 0, 0])
    frame["bones"][skeleton.ids["LeftUpLeg"]]["quaternion"] = delta.as_quat()
    frame["received"] = time.monotonic()
    actual = mapping.convert(frame)["smpl_pose"][0]
    basis = R.from_matrix(calibration["smpl_world_basis"])
    expected = (basis.inv() * delta * basis).as_rotvec()
    np.testing.assert_allclose(actual, expected, atol=1e-5)


def test_stream_starts_only_with_full_buffer_and_open_hands(sample):
    skeleton, frame = sample
    publisher = Publisher()
    session = Session(publisher)
    session.mapping = Mapping(skeleton, skeleton.reference(frame))
    now = time.monotonic()
    session.state = "standing"
    session.started = now - 10
    session.feedback_at = session.lowstate_at = now
    session.source(frame, skeleton, now)
    session.resume(now)
    for i in range(15):
        frame["received"] = time.monotonic()
        session.tick(frame, frame["received"])
        if i < 14:
            assert not any(m.startswith(b"pose") for m in publisher.messages)
            assert session.state == "arming"
    poses = [m for m in publisher.messages if m.startswith(b"pose")]
    assert len(poses) == 1
    header = json.loads(poses[0][4 : 4 + 1280].rstrip(b"\0"))
    assert header["v"] == 3
    fields = {f["name"]: f for f in header["fields"]}
    assert fields["smpl_pose"]["shape"] == [15, 21, 3]
    assert fields["smpl_joints"]["shape"] == [15, 24, 3]
    np.testing.assert_array_equal(np.frombuffer(poses[0][-56:], dtype="<f4"), np.zeros(14))
    assert session.state == "pose"
    session.pause("test")
    assert session.state == "standing"


def test_frame_arriving_during_snapshot_does_not_clear_calibration(sample, monkeypatch):
    skeleton, frame = sample
    session = Session(Publisher())
    mapping = object()
    session.mapping = mapping
    session.source_valid = True
    clock = [100.0]
    monkeypatch.setattr(time, "monotonic", lambda: clock[0])

    class Receiver:
        def latest(self):
            # NatNet callback runs after the start of the poll iteration.
            clock[0] += 0.001
            frame["received"] = clock[0]
            return frame

    snapshot, now = session.poll_source(Receiver(), skeleton)
    assert now >= snapshot["received"]
    assert session.source_valid
    assert session.mapping is mapping
    assert session.state == "idle"


def test_poll_still_rejects_genuinely_stale_frames(sample):
    skeleton, frame = sample
    session = Session(Publisher())
    session.mapping = object()
    session.source_valid = True
    frame["received"] -= 1

    class Receiver:
        def latest(self):
            return frame

    session.poll_source(Receiver(), skeleton)
    assert not session.source_valid
    assert session.mapping is None
