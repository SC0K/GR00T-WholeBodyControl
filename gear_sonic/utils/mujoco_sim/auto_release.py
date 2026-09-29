"""One-shot suspension release after SONIC starts publishing control feedback."""
import msgpack
import numpy as np
import zmq


class AutoRelease:
    def __init__(self):
        self.socket = zmq.Context.instance().socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.SUBSCRIBE, b'g1_debug')
        self.socket.connect('tcp://127.0.0.1:5557')
        self.released = False

    def poll(self, band):
        if self.released or not self.socket.poll(0):
            return
        packet = self.socket.recv()
        try:
            state = msgpack.unpackb(packet[len(b'g1_debug'):], raw=False)
            joints = np.asarray(state['body_q_measured'], dtype=float)
            if joints.shape != (29,) or not np.isfinite(joints).all():
                return
        except (ValueError, TypeError, KeyError, msgpack.UnpackException):
            return
        band.enable = False
        self.released = True
        self.close()
        print('SONIC control active: suspension released automatically. Do not press 9.', flush=True)

    def close(self):
        self.socket.close(linger=0)
