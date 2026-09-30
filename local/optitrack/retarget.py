"""Calibrated OptiTrack global bone rotations -> SONIC's canonical SMPL features.

Motive must stream global positions in meters with Z up. Calibration is a T-pose.
Source bone reference axes are measured at calibration, never assumed to be SMPL.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R
import torch

from gear_sonic.scripts.pico_manager_thread_server import process_smpl_joints

# SMPL's first 22 body joints. Motive has two spine segments: the extra SMPL
# spine3 inherits Spine1's orientation, preserving the neck/collar hierarchy.
NAMES = (
    "Hips",
    "LeftUpLeg",
    "RightUpLeg",
    "Spine",
    "LeftLeg",
    "RightLeg",
    "Spine1",
    "LeftFoot",
    "RightFoot",
    "Spine1",
    "LeftToeBase",
    "RightToeBase",
    "Neck",
    "LeftShoulder",
    "RightShoulder",
    "Head",
    "LeftArm",
    "RightArm",
    "LeftForeArm",
    "RightForeArm",
    "LeftHand",
    "RightHand",
)
PARENTS = (-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19)
# SMPL +X left, +Y up, +Z forward -> robot +X forward, +Y left, +Z up.
SMPL_TO_ROBOT = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
Y_TO_Z = R.from_euler("x", 90, degrees=True)


class Skeleton:
    def __init__(self, description):
        self.description = description
        self.ids = {}
        for bone in description:
            for name in set(NAMES):
                if bone["name"].endswith("_" + name):
                    if name in self.ids:
                        raise ValueError(f"Ambiguous bone name: {name}")
                    self.ids[name] = bone["id"]
        missing = set(NAMES) - self.ids.keys()
        if missing:
            raise ValueError(f"Missing required full-body bones: {sorted(missing)}")
        self.signature = [(b["id"], b["parent_id"], b["name"]) for b in description]
        self.parents = {b["id"]: b["parent_id"] for b in description}
        self.lengths = {b["id"]: np.linalg.norm(b["offset"]) for b in description}

    def validate(self, frame, now=None):
        now = time.monotonic() if now is None else now
        if frame["frame"] < 0 or not 0 <= now - frame["received"] < 0.35:
            raise ValueError("No fresh NatNet frame")
        bones = frame["bones"]
        for name, bid in self.ids.items():
            if bid not in bones:
                raise ValueError(f"Missing bone: {name}")
            bone = bones[bid]
            pos = np.asarray(bone["position"])
            quat = np.asarray(bone["quaternion"])
            if pos.shape != (3,) or quat.shape != (4,) or not np.isfinite(np.r_[pos, quat]).all():
                raise ValueError(f"Invalid pose: {name}")
            if not 0.95 < np.linalg.norm(quat) < 1.05 or np.max(np.abs(pos)) > 50:
                raise ValueError(f"Invalid quaternion/position: {name}")
            parent = self.parents[bid]
            if parent in bones and self.lengths[bid] > 0.02:
                length = np.linalg.norm(pos - np.asarray(bones[parent]["position"]))
                if not 0.65 * self.lengths[bid] < length < 1.35 * self.lengths[bid]:
                    raise ValueError(f"Invalid bone length/global coordinates: {name}")
        return bones

    def position(self, bones, name):
        return np.asarray(bones[self.ids[name]]["position"], dtype=float)

    def rotations(self, bones):
        return R.from_quat([bones[self.ids[name]]["quaternion"] for name in NAMES])

    def reference(self, frame):
        bones = self.validate(frame)
        hip = self.position(bones, "Hips")
        head = self.position(bones, "Head")
        up = np.array([0.0, 0.0, 1.0])
        left = self.position(bones, "LeftUpLeg") - self.position(bones, "RightUpLeg")
        left[2] = 0
        if np.linalg.norm(left) < 0.08:
            raise ValueError("Invalid hip separation")
        left /= np.linalg.norm(left)
        forward = np.cross(left, up)
        if head[2] - hip[2] < 0.35 or np.linalg.norm((head - hip)[:2]) > 0.2:
            raise ValueError("Calibration requires upright standing with Z-up global coordinates")
        for side, direction in [("Left", left), ("Right", -left)]:
            arm = self.position(bones, side + "Hand") - self.position(bones, side + "Arm")
            leg = hip - self.position(bones, side + "Foot")
            if np.linalg.norm(arm) < 0.3 or np.dot(arm / np.linalg.norm(arm), direction) < 0.93:
                raise ValueError("Hold a T-pose: both arms straight sideways and level")
            if leg[2] < 0.5 or np.linalg.norm(leg[:2]) > 0.35:
                raise ValueError("Stand upright with straight legs for calibration")
        heading = np.column_stack((forward, left, up))
        return {
            "schema_version": 1,
            "signature": self.signature,
            "reference_quaternions_xyzw": self.rotations(bones).as_quat().tolist(),
            "smpl_world_basis": (heading @ SMPL_TO_ROBOT).tolist(),
            "reference_frame": frame["frame"],
        }


class Mapping:
    def __init__(self, skeleton, calibration):
        if [list(s) for s in skeleton.signature] != [list(s) for s in calibration["signature"]]:
            raise ValueError("Skeleton definition changed; recalibrate")
        self.skeleton = skeleton
        self.reference = R.from_quat(calibration["reference_quaternions_xyzw"])
        self.basis = R.from_matrix(calibration["smpl_world_basis"])

    def convert(self, frame):
        bones = self.skeleton.validate(frame)
        # Map each measured reference frame onto the same SMPL T-pose basis.
        world = self.skeleton.rotations(bones) * self.reference.inv() * self.basis
        local = [world[parent].inv() * world[i] for i, parent in enumerate(PARENTS) if i]
        body = np.zeros((1, 69), dtype=np.float32)
        body[0, :63] = np.stack([r.as_rotvec() for r in local]).ravel()
        # Hands remain open and wrist rotations neutral until separately validated.
        body[0, 19 * 3 : 21 * 3] = 0
        global_y_up = (Y_TO_Z.inv() * world[0]).as_rotvec().astype(np.float32)[None]
        with torch.inference_mode():
            features = process_smpl_joints(
                torch.from_numpy(body), torch.from_numpy(global_y_up), torch.zeros(1, 3)
            )
        return {
            "smpl_pose": features["smpl_pose"].numpy()[0, :63].reshape(21, 3),
            "smpl_joints": features["smpl_joints_local"].numpy()[0],
            "body_quat_w": features["global_orient_quat"].numpy()[0],
            "joint_pos": np.zeros(29, dtype=np.float32),
            "joint_vel": np.zeros(29, dtype=np.float32),
        }
