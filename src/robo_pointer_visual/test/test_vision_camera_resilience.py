import types
import numpy as np
import rclpy
from rclpy.parameter import Parameter


class _DummyThread:
    def __init__(self, *args, **kwargs):
        pass

    def start(self):
        # Do not start background work in tests
        return None

    def join(self, *args, **kwargs):
        return None


class _FakeCap:
    instances = []

    def __init__(self, src, backend=None):
        self.src = src
        self.backend = backend
        self._opened = True
        self.props = {}
        _FakeCap.instances.append(self)

    def isOpened(self):
        return self._opened

    def set(self, prop, value):
        self.props[prop] = value
        return True

    def get(self, prop):
        return self.props.get(prop, 0)

    def read(self):
        # Return a small dummy frame
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        return True, frame

    def release(self):
        self._opened = False


class _FakeYOLO:
    def __init__(self, name):
        self.names = ['person', 'bottle', 'cup']

    def to(self, device):
        return self

    def half(self):
        return self

    def __call__(self, frame, verbose=False, conf=0.5):
        # Minimal result with no detections
        r = types.SimpleNamespace()
        r.boxes = []
        return [r]


def test_camera_reopen_on_param_update(monkeypatch):
    # Patch heavy/IO dependencies
    import cv2
    import ultralytics
    monkeypatch.setattr(cv2, 'VideoCapture', _FakeCap)
    monkeypatch.setattr(ultralytics, 'YOLO', _FakeYOLO)
    monkeypatch.setattr('threading.Thread', _DummyThread)

    rclpy.init()
    try:
        from robo_pointer_visual.vision_node import VisionNode
        node = VisionNode()
        try:
            # Initially one VideoCapture instance created on first _configure_camera call
            # (not invoked automatically because we stubbed threads), so call it manually
            node._configure_camera()
            initial_instances = len(_FakeCap.instances)

            # Update backend -> should trigger reopen
            res = node._on_set_parameters([
                Parameter('camera_backend', Parameter.Type.STRING, 'v4l2'),
            ])
            assert res.successful
            assert len(_FakeCap.instances) >= initial_instances + 1

            # Update camera_index -> should also trigger reopen and change source
            res = node._on_set_parameters([
                Parameter('camera_index', Parameter.Type.STRING, '/dev/camera_robot'),
            ])
            assert res.successful
            assert node.camera_capture_source == '/dev/camera_robot'
            assert len(_FakeCap.instances) >= initial_instances + 2
        finally:
            node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

