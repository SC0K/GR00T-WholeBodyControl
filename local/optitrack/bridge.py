"""OptiTrack full-body -> SONIC, restricted to local MuJoCo simulation."""

import argparse
from collections import deque
import json
from pathlib import Path
import select
import sys
import time

import msgpack
import numpy as np
from scipy.spatial.transform import Rotation as R
import zmq

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)
from retarget import Mapping, Skeleton

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / ".venv_teleop/mocap"))
import _natnet

CALIBRATION = ROOT / ".venv_teleop/mocap/calibration.json"


def calibrate(receiver, skeleton, path):
    print("Hold a T-pose: upright, arms straight sideways, palms down. Capturing for 1 second.", flush=True)
    references = []
    last_frame = None
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        frame = receiver.latest()
        if frame["frame"] != last_frame:
            references.append(skeleton.reference(frame))
            last_frame = frame["frame"]
        time.sleep(0.01)
    if len(references) < 30:
        raise ValueError("Not enough fresh frames for calibration")
    quats = np.array([r["reference_quaternions_xyzw"] for r in references])
    means = np.array([R.from_quat(quats[:, j]).mean().as_quat() for j in range(22)])
    for j in range(22):
        if np.max((R.from_quat(quats[:, j]) * R.from_quat(means[j]).inv()).magnitude()) > 0.12:
            raise ValueError("Moved during calibration; hold still and retry")
    result = references[-1]
    result["reference_quaternions_xyzw"] = means.tolist()
    result["created_at"] = time.time()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(f"Calibration saved: {path}", flush=True)
    return Mapping(skeleton, result)


class Session:
    """Explicit start/resume; source loss clears calibration, feedback loss stops."""

    def __init__(self, pub):
        self.pub = pub
        self.state = "idle"
        self.started = 0.0
        self.feedback_at = 0.0
        self.lowstate_at = 0.0
        self.last_source_change = 0.0
        self.last_source_number = None
        self.last_source_timestamp = None
        self.source_valid = False
        self.mapping = None
        self.buffer = deque(maxlen=15)
        self.frame_index = 0
        self.heading = 0.0
        self.last_pose = None

    def command(self, start=False, stop=False, planner=True):
        self.pub.send(build_command_message(start, stop, planner))

    def stop(self, reason):
        self.command(stop=True)
        self.state = "idle"
        self.buffer.clear()
        self.last_pose = None
        print("STOP:", reason, flush=True)

    def pause(self, reason, invalidate=False):
        if self.state in ("pose", "arming"):
            self.command(planner=True)
            self.state = "standing"
        self.buffer.clear()
        self.last_pose = None
        if invalidate:
            self.mapping = None
        print("PAUSED:", reason, flush=True)

    def poll_source(self, receiver, skeleton):
        # A callback can replace the frame while latest() acquires its lock.
        # Sample the clock after that snapshot so a fresh frame cannot have
        # a negative age and spuriously invalidate calibration.
        frame = receiver.latest()
        now = time.monotonic()
        self.source(frame, skeleton, now)
        return frame, now

    def source(self, frame, skeleton, now):
        try:
            skeleton.validate(frame, now)
            if self.last_source_timestamp is not None and frame["timestamp"] < self.last_source_timestamp:
                self.last_source_number = None
                self.last_source_timestamp = frame["timestamp"]
                raise ValueError("Motive timestamp reset; recalibrate")
            if frame["frame"] != self.last_source_number:
                self.last_source_change = now
                self.last_source_number = frame["frame"]
                self.last_source_timestamp = frame["timestamp"]
            if now - self.last_source_change > 0.35:
                raise ValueError("Motive frame counter stopped")
            self.source_valid = True
        except ValueError as error:
            if self.source_valid or self.state in ("pose", "arming"):
                self.pause(str(error), invalidate=True)
            self.source_valid = False

    def start(self, now):
        if not self.source_valid or self.mapping is None:
            raise ValueError("Fresh skeleton and T-pose calibration required")
        if now - self.lowstate_at > 0.35:
            raise ValueError("Start the local MuJoCo simulator first")
        if self.state != "idle":
            raise ValueError("Already started; use Enter to resume pose streaming")
        self.started = now
        self.state = "standing"
        self.command(start=True, planner=True)
        print("Standing control requested; wait at least 5 seconds, lower arms, then Enter.", flush=True)

    def resume(self, now):
        if self.state != "standing" or now - self.started < 5:
            raise ValueError("Start standing control and wait at least 5 seconds")
        if not self.source_valid or self.mapping is None or now - self.feedback_at > 0.35:
            raise ValueError("Fresh tracking, calibration and controller feedback required")
        self.buffer.clear()
        self.last_pose = None
        self.state = "arming"
        print("Filling the 15-frame pose buffer before switching control", flush=True)

    def tick(self, frame, now):
        if self.state == "idle":
            return
        if now - self.lowstate_at > 0.35:
            self.stop("Robot state lost")
            return
        if now - self.started > 5 and now - self.feedback_at > 1:
            self.stop("Controller feedback lost")
            return
        if self.state in ("pose", "arming"):
            if not self.source_valid or self.mapping is None:
                self.pause("Tracking unavailable", invalidate=True)
            else:
                pose = self.mapping.convert(frame)
                if self.last_pose is not None:
                    previous = R.from_rotvec(self.last_pose["smpl_pose"])
                    current = R.from_rotvec(pose["smpl_pose"])
                    root_change = (
                        R.from_quat(pose["body_quat_w"][[1, 2, 3, 0]])
                        * R.from_quat(self.last_pose["body_quat_w"][[1, 2, 3, 0]]).inv()
                    ).magnitude()
                    if max(np.max((current * previous.inv()).magnitude()), root_change) > 0.6:
                        self.pause("Abrupt skeleton rotation; recalibrate", invalidate=True)
                if self.state in ("pose", "arming"):
                    self.last_pose = pose
                    quat = pose["body_quat_w"]
                    self.heading = R.from_quat(quat[[1, 2, 3, 0]]).as_euler("xyz")[2]
                    pose["frame_index"] = np.array(self.frame_index, dtype=np.int64)
                    self.frame_index += 1
                    self.buffer.append(pose)
                    data = {key: np.stack([p[key] for p in self.buffer]) for key in pose}
                    data["left_hand_joints"] = np.zeros(7, dtype=np.float32)
                    data["right_hand_joints"] = np.zeros(7, dtype=np.float32)
                    if len(self.buffer) == self.buffer.maxlen:
                        self.pub.send(pack_pose_message(data))
                        if self.state == "arming":
                            self.command(planner=False)
                            self.state = "pose"
                            print("Full-body pose streaming enabled", flush=True)
        if self.state in ("standing", "arming"):
            self.pub.send(
                build_planner_message(
                    mode=0,
                    movement=[0, 0, 0],
                    facing=[np.cos(self.heading), np.sin(self.heading), 0],
                    speed=0.0,
                    height=0.78,
                    left_hand_position=np.zeros(7),
                    right_hand_position=np.zeros(7),
                )
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--calibrate", action="store_true", help="Capture a T-pose and exit; sends no robot commands"
    )
    parser.add_argument("--monitor", type=float, metavar="SECONDS", help="Inspect skeleton frames only")
    parser.add_argument("--calibration", type=Path, default=CALIBRATION)
    args = parser.parse_args()
    config = json.loads((ROOT / "local/optitrack/setup.json").read_text())
    if config["deployment"] != "simulation_only":
        raise ValueError("This adapter is restricted to simulation")
    source = config["source"]
    receiver = _natnet.Receiver(source["server_ip"], source["client_interface_ip"], source["skeleton_id"])
    context = pub = feedback = subscriber = session = None
    try:
        skeleton = Skeleton(receiver.description())
        time.sleep(0.15)
        if args.monitor is not None:
            end = time.monotonic() + args.monitor
            frames, last = 0, None
            while time.monotonic() < end:
                frame = receiver.latest()
                skeleton.validate(frame)
                if frame["frame"] != last:
                    frames += 1
                    last = frame["frame"]
                time.sleep(0.005)
            print(f"Valid full-body samples: {frames}; last frame: {last}")
            return
        if args.calibrate:
            calibrate(receiver, skeleton, args.calibration)
            return
        if not sys.stdin.isatty():
            raise ValueError("Run interactively; use --monitor for receive-only validation")
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        context = zmq.Context()
        pub = context.socket(zmq.PUB)
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(zmq.SNDHWM, 2)
        pub.bind("tcp://127.0.0.1:5556")
        feedback = context.socket(zmq.SUB)
        feedback.setsockopt(zmq.LINGER, 0)
        feedback.setsockopt(zmq.CONFLATE, 1)
        feedback.setsockopt(zmq.SUBSCRIBE, b"g1_debug")
        feedback.connect("tcp://127.0.0.1:5557")
        session = Session(pub)
        if args.calibration.exists():
            session.mapping = Mapping(skeleton, json.loads(args.calibration.read_text()))
        ChannelFactoryInitialize(0, "lo")
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)

        def lowstate(message):
            if np.isfinite([m.q for m in message.motor_state[:29]]).all():
                session.lowstate_at = time.monotonic()

        subscriber.Init(lowstate, 10)
        print(
            "SIMULATION ONLY. Commands + Enter: c calibrate T-pose; s start standing; Enter stream; p pause; o stop; q quit.",
            flush=True,
        )
        last_status = 0.0
        deadline = time.monotonic()
        while True:
            frame, now = session.poll_source(receiver, skeleton)
            if feedback.poll(0):
                try:
                    message = msgpack.unpackb(feedback.recv()[len(b"g1_debug") :], raw=False)
                    if "body_q_measured" in message and np.isfinite(message["body_q_measured"]).all():
                        session.feedback_at = now
                except (ValueError, TypeError, msgpack.UnpackException):
                    pass
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip() == "q":
                    break
                command = line.strip().lower()
                try:
                    if command == "c":
                        if session.state != "idle":
                            raise ValueError("Stop control before calibrating")
                        session.mapping = calibrate(receiver, skeleton, args.calibration)
                    elif command == "s":
                        session.start(now)
                    elif command == "":
                        session.resume(now)
                    elif command == "p":
                        session.pause("User pause")
                    elif command == "o":
                        session.stop("User stop")
                    else:
                        print(
                            f"Unknown command {command!r}. Use c to calibrate, s to start standing, "
                            "Enter to stream, p to pause, o to stop, or q to quit.",
                            flush=True,
                        )
                except ValueError as error:
                    print("Not started:", error, flush=True)
            try:
                session.tick(frame, now)
            except ValueError as error:
                session.pause(str(error), invalidate=True)
            if now - last_status > 5:
                print(
                    f"{session.state}: tracking={session.source_valid}, calibrated={session.mapping is not None}, frame={frame['frame']}",
                    flush=True,
                )
                last_status = now
            deadline += 0.02
            if deadline < time.monotonic() - 0.02:
                deadline = time.monotonic()
            time.sleep(max(0.0, deadline - time.monotonic()))
    finally:
        if session:
            for _ in range(5):
                session.command(stop=True)
                time.sleep(0.02)
        if subscriber:
            subscriber.Close()
        receiver.close()
        if pub:
            pub.close()
        if feedback:
            feedback.close()
        if context:
            context.term()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
