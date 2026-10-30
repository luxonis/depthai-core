import os
import time

import depthai as dai
import pytest


def test_camera_stats_optionals_and_precision():
    stats = dai.CameraStats()
    assert stats.status == dai.CameraStats.Status.UNAVAILABLE
    assert stats.socket == dai.CameraBoardSocket.AUTO
    assert stats.completedCaptures is None
    assert stats.cslErrorEvents is None
    stats.completedCaptures = 0
    stats.cslErrorEvents = (1 << 64) - 1
    stats.timestampNs = (1 << 53) + 1
    stats.sessionId = "test-session"
    values = dai.VectorCameraStats([stats])
    assert values[0].completedCaptures == 0
    assert values[0].cslErrorEvents == (1 << 64) - 1
    assert values[0].timestampNs == (1 << 53) + 1
    values[0].cslErrorEvents = None
    assert values[0].cslErrorEvents is None


@pytest.mark.skipif(os.environ.get("DEPTHAI_BINDINGS_DEVICE_TESTS") != "1",
                    reason="Requires matching firmware and two RVC4 sensors")
def test_camera_stats_two_sensors():
    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        if device.getPlatform() != dai.Platform.RVC4:
            pytest.skip("Requires RVC4")
        sockets = [camera.socket for camera in device.getConnectedCameraFeatures()]
        if len(set(sockets)) < 2:
            pytest.skip("Requires two sensors")
        sockets = sockets[:2]
        queues = [
            pipeline.create(dai.node.Camera).build(socket, sensorFps=15.0)
            .requestOutput((320, 240), fps=15.0).createOutputQueue(1, False)
            for socket in sockets
        ]
        pipeline.start()
        previous = {}
        advanced = set()
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and len(advanced) < 2:
            for queue in queues:
                queue.tryGet()
            readings = device.getCameraStats()
            assert isinstance(readings, dai.VectorCameraStats)
            for current in readings:
                if current.socket not in sockets or current.status != dai.CameraStats.Status.AVAILABLE:
                    continue
                assert isinstance(current.completedCaptures, int)
                old = previous.get(current.socket)
                if old and old.sessionId == current.sessionId:
                    assert current.completedCaptures >= old.completedCaptures
                    if current.completedCaptures > old.completedCaptures:
                        advanced.add(current.socket)
                previous[current.socket] = current
            time.sleep(0.02)
        assert advanced == set(sockets)
