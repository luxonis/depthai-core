# -*- coding: utf-8 -*-
"""Python catalog P1-P4 and P6-P9 for dai.node.ReplayVideo. There is no P5."""

import os
import tempfile
import time
from datetime import timedelta
from pathlib import Path

import depthai as dai
import numpy as np
import pytest

_DEVICE_ENV = ("DEPTHAI_REPLAY", "DEPTHAI_RECORD", "DEPTHAI_REPLAY_DECODE_MODE")


def nv12_plain(width, height, y_value, u_value, v_value):
    y_size = width * height
    data = np.empty(y_size + y_size // 2, dtype=np.uint8)
    data[:y_size] = np.uint8(y_value)
    data[y_size::2] = np.uint8(u_value)
    data[y_size + 1 :: 2] = np.uint8(v_value)
    return data


def nv12_gradient(width, height, index):
    """Same bytes as replay_test::frameBytes GRADIENT for NV12."""
    x = np.arange(width, dtype=np.uint16)
    y = np.arange(height, dtype=np.uint16)[:, None]
    plane_y = np.bitwise_and(x + y + np.uint16(index), 255).astype(np.uint8)
    uv_rows = height // 2
    chroma_y = np.arange(uv_rows, dtype=np.uint16)
    uv = np.empty((uv_rows, width), dtype=np.uint8)
    uv[:, 0::2] = np.bitwise_and(x[0::2], 255).astype(np.uint8)
    uv[:, 1::2] = np.bitwise_and(chroma_y * 2, 255).astype(np.uint8)[:, None]
    return np.concatenate((plane_y.reshape(-1), uv.reshape(-1)))


def transformation(width, height):
    """T(width x height): intrinsics [[w/2, 0, w/2], [0, w/2, h/2], [0, 0, 1]]."""
    half_w = float(width) / 2.0
    half_h = float(height) / 2.0
    intrinsics = (
        (half_w, 0.0, half_w),
        (0.0, half_w, half_h),
        (0.0, 0.0, 1.0),
    )
    return dai.ImgTransformation(int(width), int(height), intrinsics)


def make_dai_frame(path, width, height, sequence, ts_ms, data):
    """Same fields as replay_test::makeDaiFrame, including the transformation."""
    frame = dai.ImgFrame()
    frame.setType(dai.ImgFrame.Type.NV12)
    frame.setWidth(int(width))
    frame.setHeight(int(height))
    frame.setData(np.ascontiguousarray(data, dtype=np.uint8))
    frame.setSequenceNum(int(sequence))
    when = timedelta(milliseconds=int(ts_ms))
    frame.setTimestamp(when)
    frame.setTimestampDevice(when)
    frame.setTransformation(transformation(width, height))
    path.parent.mkdir(parents=True, exist_ok=True)
    frame.save(str(path))


def make_dai_dir(directory, width, height, sequences, timestamps_ms, data_for):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    frames = []
    for sequence, ts_ms in zip(sequences, timestamps_ms):
        data = np.ascontiguousarray(data_for(sequence), dtype=np.uint8)
        make_dai_frame(directory / f"cam_{int(sequence)}.dai", width, height, sequence, ts_ms, data)
        frames.append(data)
    return frames


def make_gradient_dir(directory, width, height, count):
    sequences = list(range(count))
    timestamps = [index * 100 for index in sequences]
    return make_dai_dir(directory, width, height, sequences, timestamps, lambda sequence: nv12_gradient(width, height, sequence))


def assert_nv12_rows(frame, expected, width, height):
    data = np.asarray(frame.getData())
    expected = np.asarray(expected, dtype=np.uint8).reshape(-1)
    stride = int(frame.getStride())
    assert stride >= width
    y_base = int(frame.fb.p1Offset)
    uv_base = int(frame.fb.p2Offset)
    if uv_base <= y_base:
        uv_base = y_base + stride * height
    for row in range(height):
        got = data[y_base + row * stride : y_base + row * stride + width]
        exp = expected[row * width : (row + 1) * width]
        assert np.array_equal(got, exp)
    chroma = width * height
    for row in range(height // 2):
        got = data[uv_base + row * stride : uv_base + row * stride + width]
        exp = expected[chroma + row * width : chroma + (row + 1) * width]
        assert np.array_equal(got, exp)


def timestamp_ms(value):
    return value // timedelta(milliseconds=1)


def take_frame(queue, timeout_s=10):
    frame = queue.get(timedelta(seconds=timeout_s))
    assert frame is not None, "timed out waiting for a replay frame"
    assert isinstance(frame, dai.ImgFrame)
    return frame


def host_replay(source, *, fps=1000, loop=False, stop_at_end=False, out_type=None, size=None, decode=None, persistent=None):
    pipeline = dai.Pipeline(False)
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setSource(str(source))
    replay.setLoop(loop)
    replay.setStopPipelineAtEnd(stop_at_end)
    if fps is not None:
        replay.setFps(float(fps))
    if out_type is not None:
        replay.setOutFrameType(out_type)
    if size is not None:
        replay.setSize(int(size[0]), int(size[1]))
    if decode is not None:
        replay.setDecodeMode(decode)
    if persistent is not None:
        replay.setPersistentUpload(persistent)
    queue = replay.out.createOutputQueue(16, True)
    return pipeline, replay, queue


def stop_pipeline(pipeline):
    if pipeline is not None and pipeline.isRunning():
        pipeline.stop()
        pipeline.wait()


def require_rvc4():
    infos = [info for info in dai.Device.getAllAvailableDevices() if info.platform == dai.XLinkPlatform.X_LINK_RVC4]
    if not infos:
        pytest.skip("needs an RVC4 device")
    return infos[0]


def hide_device_env():
    saved = {name: os.environ.get(name) for name in _DEVICE_ENV}
    for name in _DEVICE_ENV:
        os.environ.pop(name, None)
    return saved


def restore_env(saved):
    for name, value in saved.items():
        if value is None:
            os.environ.pop(name, None)
        else:
            os.environ[name] = value


def test_p1_dai_directory():
    """P1 mirrors B1: 3 NV12 frames, sequences 0..2, data equal per row at the stride."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p1-")
    pipeline = None
    try:
        frames = make_gradient_dir(directory.name, 64, 48, 3)
        pipeline, _replay, queue = host_replay(directory.name)
        pipeline.start()
        for sequence, expected in enumerate(frames):
            frame = take_frame(queue)
            assert frame.getSequenceNum() == sequence
            assert frame.getType() == dai.ImgFrame.Type.NV12
            assert_nv12_rows(frame, expected, 64, 48)
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_p2_odd_size():
    """P2 mirrors A6: setSize(321, 240) fails at start and the text contains size."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p2-")
    pipeline = None
    try:
        make_gradient_dir(directory.name, 64, 48, 3)
        pipeline, _replay, _queue = host_replay(directory.name, size=(321, 240))
        with pytest.raises(RuntimeError, match="size"):
            pipeline.start()
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_p3_loop_metadata():
    """P3 mirrors C15. fps 1000 is the fast-playback sentinel; the pass boundary stays the recorded 150 ms mean."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p3-")
    pipeline = None
    try:
        sequences = (10, 11, 12)
        timestamps = (1000, 1100, 1300)
        make_dai_dir(
            directory.name,
            64,
            48,
            sequences,
            timestamps,
            lambda sequence: nv12_gradient(64, 48, sequence),
        )
        pipeline, _replay, queue = host_replay(directory.name, loop=True)
        pipeline.start()
        expected = (
            (10, 1000),
            (11, 1100),
            (12, 1300),
            (13, 1450),
            (14, 1550),
            (15, 1750),
            (16, 1900),
        )
        for sequence, ts_ms in expected:
            frame = take_frame(queue)
            assert frame.getSequenceNum() == sequence
            assert timestamp_ms(frame.getTimestampDevice()) == ts_ms
            assert frame.getTimestamp() == frame.getTimestampDevice()
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_p4_gray8():
    """P4 mirrors D2: GRAY8 output of Plain(100, 128, 128) is all 100."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p4-")
    pipeline = None
    try:
        plain = nv12_plain(64, 48, 100, 128, 128)
        make_dai_dir(directory.name, 64, 48, (0,), (0,), lambda _sequence: plain)
        pipeline, _replay, queue = host_replay(directory.name, out_type=dai.ImgFrame.Type.GRAY8)
        pipeline.start()
        frame = take_frame(queue)
        assert frame.getType() == dai.ImgFrame.Type.GRAY8
        assert frame.getWidth() == 64
        assert frame.getHeight() == 48
        assert np.all(np.asarray(frame.getData()) == 100)
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_p6_fps_pacing():
    """P6 mirrors E1: gaps at least 90 ms, mean gap 100 ms within 15%, span 900 ms within 25%."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p6-")
    pipeline = None
    try:
        make_gradient_dir(directory.name, 64, 48, 10)
        pipeline, _replay, queue = host_replay(directory.name, fps=10)
        pipeline.start()
        arrivals = []
        for sequence in range(10):
            frame = take_frame(queue, timeout_s=10)
            arrivals.append(time.monotonic())
            assert frame.getSequenceNum() == sequence
        gaps_ms = [(arrivals[index + 1] - arrivals[index]) * 1000.0 for index in range(9)]
        assert min(gaps_ms) >= 90.0
        mean_gap = sum(gaps_ms) / len(gaps_ms)
        assert abs(mean_gap - 100.0) <= 15.0
        span_ms = (arrivals[-1] - arrivals[0]) * 1000.0
        assert abs(span_ms - 900.0) <= 0.25 * 900.0
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_p7_host_decoded_cache():
    """P7 mirrors G1: one decoded host-cache entry after a persistent PRE_DECODE run."""
    directory = tempfile.TemporaryDirectory(prefix="dai-p7-")
    pipeline = None
    dai.node.ReplayVideo.clearReplayCache()
    try:
        make_gradient_dir(directory.name, 64, 48, 100)
        pipeline, _replay, _queue = host_replay(
            directory.name,
            decode=dai.ReplayDecodeMode.PRE_DECODE,
            persistent=True,
        )
        pipeline.start()
        # Decode commits the DECODED entry when the pass finishes. Wait for that, then stop, and require it to remain.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if len(dai.node.ReplayVideo.getReplayCacheInfo().entries) >= 1:
                break
            time.sleep(0.05)
        pipeline.stop()
        pipeline.wait()
        info = dai.node.ReplayVideo.getReplayCacheInfo()
        assert len(info.entries) == 1
        # The spec names CacheEntryKind; the bound enum is dai.ReplayCacheKind.
        assert info.entries[0].kind == dai.ReplayCacheKind.DECODED
    finally:
        stop_pipeline(pipeline)
        dai.node.ReplayVideo.clearReplayCache()
        directory.cleanup()


@pytest.mark.rvc4
def test_p8_device_source_cache():
    """P8 mirrors VA1: one SOURCE entry, then clearReplayCache empties it."""
    info = require_rvc4()
    saved_env = hide_device_env()
    directory = tempfile.TemporaryDirectory(prefix="dai-p8-")
    device = None
    pipeline = None
    limit = None
    try:
        make_gradient_dir(directory.name, 64, 48, 3)
        device = dai.Device(info)
        if device.getPlatform() != dai.Platform.RVC4:
            pytest.skip("needs an RVC4 device")
        limit = device.getReplayCacheInfo().limitBytes
        device.clearReplayCache()
        pipeline = dai.Pipeline(device)
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setSource(str(directory.name))
        replay.setLoop(False)
        replay.setStopPipelineAtEnd(False)
        replay.setFps(1000)
        replay.setPersistentUpload(True)
        queue = replay.out.createOutputQueue(16, True)
        pipeline.start()
        for sequence in range(3):
            frame = take_frame(queue, timeout_s=30)
            assert frame.getSequenceNum() == sequence
        pipeline.stop()
        pipeline.wait()
        entries = device.getReplayCacheInfo().entries
        assert len(entries) == 1
        assert entries[0].kind == dai.ReplayCacheKind.SOURCE
        device.clearReplayCache()
        assert len(device.getReplayCacheInfo().entries) == 0
    finally:
        stop_pipeline(pipeline)
        if device is not None:
            try:
                device.clearReplayCache()
                if limit is not None:
                    device.setReplayCacheLimit(limit)
            except Exception:
                pass
            device.close()
        directory.cleanup()
        restore_env(saved_env)


@pytest.mark.rvc4
def test_p9_camera_replay():
    """P9 mirrors VC8: camera.build(CAM_A, replay) and 10 frames, tsDevice 0..900 ms, NV12."""
    info = require_rvc4()
    saved_env = hide_device_env()
    directory = tempfile.TemporaryDirectory(prefix="dai-p9-")
    device = None
    pipeline = None
    try:
        make_gradient_dir(directory.name, 1920, 1080, 10)
        device = dai.Device(info)
        if device.getPlatform() != dai.Platform.RVC4:
            pytest.skip("needs an RVC4 device")
        try:
            device.clearReplayCache()
        except Exception:
            pass
        pipeline = dai.Pipeline(device)
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setSource(str(directory.name))
        replay.setLoop(False)
        replay.setStopPipelineAtEnd(False)
        replay.setFps(1000)
        camera = pipeline.create(dai.node.Camera)
        camera = camera.build(dai.CameraBoardSocket.CAM_A, replay)
        assert replay.getOutFrameType() == dai.ImgFrame.Type.NV12
        output = camera.requestOutput((1920, 1080), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 30)
        queue = output.createOutputQueue(16, True)
        pipeline.start()
        deadline = time.monotonic() + 60
        frames = []
        while len(frames) < 10:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame = queue.get(timedelta(seconds=remaining))
            if frame is None:
                break
            frames.append(frame)
        assert len(frames) == 10
        for index, frame in enumerate(frames):
            assert isinstance(frame, dai.ImgFrame)
            assert frame.getType() == dai.ImgFrame.Type.NV12
            assert timestamp_ms(frame.getTimestampDevice()) == index * 100
    finally:
        stop_pipeline(pipeline)
        if device is not None:
            try:
                device.clearReplayCache()
            except Exception:
                pass
            device.close()
        directory.cleanup()
        restore_env(saved_env)
