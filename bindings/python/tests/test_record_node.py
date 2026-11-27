# -*- coding: utf-8 -*-
"""Python catalog PY1-PY8 for dai.node.RecordVideo. Host pipeline is Pipeline(False)."""

import os
import re
import tempfile
import time
from datetime import timedelta
from pathlib import Path

import depthai as dai
import numpy as np
import pytest

WIDTH = 64
HEIGHT = 48
IDLE_SLEEP_S = 0.3
STREAM = "cam"
RECORDING = "test"
_DAI_NAME = re.compile(rf"^{STREAM}_\d{{8,}}\.dai$")
_DEVICE_ENV = ("DEPTHAI_REPLAY", "DEPTHAI_RECORD", "DEPTHAI_REPLAY_DECODE_MODE")


def nv12_plain(width, height, y_value, u_value, v_value):
    y_size = width * height
    data = np.empty(y_size + y_size // 2, dtype=np.uint8)
    data[:y_size] = np.uint8(y_value)
    data[y_size::2] = np.uint8(u_value)
    data[y_size + 1 :: 2] = np.uint8(v_value)
    return data


def make_record_frame(n, sequence=None):
    """I(64x48, n) fields the Python catalog sets: type, size, data, sequence, timestamps."""
    if sequence is None:
        sequence = n
    frame = dai.ImgFrame()
    frame.setType(dai.ImgFrame.Type.NV12)
    frame.setWidth(WIDTH)
    frame.setHeight(HEIGHT)
    frame.setData(nv12_plain(WIDTH, HEIGHT, n % 256, 128, 128))
    frame.setSequenceNum(int(sequence))
    when = timedelta(milliseconds=int(n) * 100)
    frame.setTimestamp(when)
    frame.setTimestampDevice(when)
    return frame


def copy_data(frame):
    return np.array(frame.getData(), copy=True)


def assert_nv12_rows(frame, expected, width=WIDTH, height=HEIGHT):
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


def host_record(directory, *, wait_for_trigger=False, pre_trigger=None, stream_fps=None):
    pipeline = dai.Pipeline(False)
    record = pipeline.create(dai.node.RecordVideo)
    record.setFormat(dai.RecordFormat.RAW_FRAMES)
    record.setRecordingName(RECORDING)
    record.setOutputDirectory(str(directory))
    record.setStreamName(STREAM)
    if wait_for_trigger:
        record.setWaitForTrigger(True)
    if pre_trigger is not None:
        record.setPreTriggerFrames(pre_trigger)
    if stream_fps is not None:
        # Float setter. setFps(unsigned) is the whole-number fps.
        record.setStreamFps(float(stream_fps))
    frames = record.input.createInputQueue()
    control = record.inputControl.createInputQueue()
    # getStatus() may be None. The status queue carries STARTED and STOPPED.
    status = record.status.createOutputQueue(16, True)
    stream = Path(directory) / RECORDING / STREAM
    return pipeline, record, frames, control, status, stream


def dai_files(stream_dir):
    stream_dir = Path(stream_dir)
    if not stream_dir.is_dir():
        return []
    return sorted(path for path in stream_dir.iterdir() if path.is_file() and _DAI_NAME.match(path.name))


def wait_dai_count(stream_dir, count, timeout_s=10):
    deadline = time.monotonic() + timeout_s
    while True:
        found = dai_files(stream_dir)
        if len(found) >= count:
            return found
        if time.monotonic() >= deadline:
            raise AssertionError(f"expected {count} .dai files, found {[path.name for path in found]}")
        time.sleep(0.01)


def assert_dai_names(stream_dir, sequences):
    found = [path.name for path in dai_files(stream_dir)]
    expected = [f"{STREAM}_{sequence:08d}.dai" for sequence in sequences]
    assert found == expected


def take_status(queue, timeout_s=10):
    message = queue.get(timedelta(seconds=timeout_s))
    assert message is not None, "timed out waiting for RecordStatus"
    return message


def status_after_stop(record, timeout_s=2):
    """pipeline.stop() closes the status queue. The last status is getStatus()."""
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = record.getStatus()
        if latest is not None and latest.state == dai.RecordStatus.State.STOPPED:
            return latest
        time.sleep(0.01)
    assert latest is not None, "getStatus() was empty after stop"
    return latest


def assert_started(status, segment):
    assert status.state == dai.RecordStatus.State.STARTED
    assert status.segment == segment
    assert status.frameCount == 0
    assert status.firstSequenceNum == -1
    assert status.lastSequenceNum == -1
    assert status.skippedFrames == 0


def assert_stopped(status, segment, frame_count, first, last, skipped):
    assert status.state == dai.RecordStatus.State.STOPPED
    assert status.segment == segment
    assert status.frameCount == frame_count
    assert status.firstSequenceNum == first
    assert status.lastSequenceNum == last
    assert status.skippedFrames == skipped


def send_idle(queue, frame):
    queue.send(frame)
    time.sleep(IDLE_SLEEP_S)


def send_recorded(queue, frame, stream_dir, count):
    queue.send(frame)
    wait_dai_count(stream_dir, count)


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


def delete_recording(device, name):
    if any(recording.name == name for recording in device.listRecordings()):
        device.deleteRecording(name)


def test_py1_raw_frames_round_trip():
    """PY1 mirrors R1 and R2: five files, sequences 0..4, row data, STOPPED(0, 5, 0, 4, 0)."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py1-")
    pipeline = None
    try:
        pipeline, record, frames, _control, status, stream = host_record(directory.name)
        pipeline.start()
        assert_started(take_status(status), 0)
        sent = []
        for n in range(5):
            frame = make_record_frame(n)
            sent.append(copy_data(frame))
            send_recorded(frames, frame, stream, n + 1)
        pipeline.stop()
        stopped = status_after_stop(record)
        assert_stopped(stopped, 0, 5, 0, 4, 0)
        assert Path(stopped.path).name == RECORDING
        assert_dai_names(stream, range(5))
        for n, path in enumerate(dai_files(stream)):
            loaded = dai.ImgFrame()
            loaded.load(str(path))
            assert loaded.getSequenceNum() == n
            assert_nv12_rows(loaded, sent[n])
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_py2_trigger_segments():
    """PY2 mirrors T2: four status messages and files 3, 4, 5, 8, 9."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py2-")
    pipeline = None
    try:
        pipeline, record, frames, control, status, stream = host_record(directory.name, wait_for_trigger=True, pre_trigger=0)
        pipeline.start()
        for n in (0, 1, 2):
            send_idle(frames, make_record_frame(n))
        control.send(dai.RecordControl.start())
        assert_started(take_status(status), 0)
        for n, count in ((3, 1), (4, 2), (5, 3)):
            send_recorded(frames, make_record_frame(n), stream, count)
        control.send(dai.RecordControl.stop())
        assert_stopped(take_status(status), 0, 3, 3, 5, 0)
        for n in (6, 7):
            send_idle(frames, make_record_frame(n))
        control.send(dai.RecordControl.start())
        assert_started(take_status(status), 1)
        for n, count in ((8, 4), (9, 5)):
            send_recorded(frames, make_record_frame(n), stream, count)
        pipeline.stop()
        assert_stopped(status_after_stop(record), 1, 2, 8, 9, 0)
        assert_dai_names(stream, (3, 4, 5, 8, 9))
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_py3_max_frames():
    """PY3 mirrors T4: RecordControl.start(2, -1) writes two files and STOPPED frameCount 2."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py3-")
    pipeline = None
    try:
        pipeline, _record, frames, control, status, stream = host_record(directory.name, wait_for_trigger=True, pre_trigger=0)
        pipeline.start()
        for n in (0, 1):
            send_idle(frames, make_record_frame(n))
        control.send(dai.RecordControl.start(2, -1))
        assert_started(take_status(status), 0)
        send_recorded(frames, make_record_frame(2), stream, 1)
        send_recorded(frames, make_record_frame(3), stream, 2)
        assert_stopped(take_status(status), 0, 2, 2, 3, 0)
        for n in (4, 5, 6):
            send_idle(frames, make_record_frame(n))
        assert_dai_names(stream, (2, 3))
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_py4_pre_trigger():
    """PY4 mirrors P1: seven files and the four P1 status messages."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py4-")
    pipeline = None
    try:
        pipeline, _record, frames, control, status, stream = host_record(directory.name, wait_for_trigger=True, pre_trigger=2)
        pipeline.start()
        for n in range(5):
            send_idle(frames, make_record_frame(n))
        control.send(dai.RecordControl.start())
        assert_started(take_status(status), 0)
        send_recorded(frames, make_record_frame(5), stream, 3)
        send_recorded(frames, make_record_frame(6), stream, 4)
        control.send(dai.RecordControl.stop())
        assert_stopped(take_status(status), 0, 4, 3, 6, 0)
        for n in (7, 8, 9):
            send_idle(frames, make_record_frame(n))
        control.send(dai.RecordControl.start(3, -1))
        assert_started(take_status(status), 1)
        send_recorded(frames, make_record_frame(10), stream, 7)
        assert_stopped(take_status(status), 1, 3, 8, 10, 0)
        assert_dai_names(stream, (3, 4, 5, 6, 8, 9, 10))
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_py5_skipped_frames():
    """PY5 mirrors K1: STOPPED.skippedFrames == 6."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py5-")
    pipeline = None
    try:
        pipeline, record, frames, _control, status, stream = host_record(directory.name)
        pipeline.start()
        assert_started(take_status(status), 0)
        for count, sequence in enumerate((0, 1, 5, 6, 10), start=1):
            send_recorded(frames, make_record_frame(sequence), stream, count)
        pipeline.stop()
        stopped = status_after_stop(record)
        assert_stopped(stopped, 0, 5, 0, 10, 6)
        assert stopped.skippedFrames == 6
        assert_dai_names(stream, (0, 1, 5, 6, 10))
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


def test_py6_invalid_fps():
    """PY6 mirrors A1: start fails and the error text contains fps."""
    directory = tempfile.TemporaryDirectory(prefix="dai-py6-")
    pipeline = None
    try:
        pipeline, _record, _frames, _control, _status, _stream = host_record(directory.name, stream_fps=0)
        with pytest.raises(RuntimeError, match="fps"):
            pipeline.start()
    finally:
        stop_pipeline(pipeline)
        directory.cleanup()


@pytest.mark.rvc4
def test_py7_device_recordings():
    """PY7 mirrors VR1 and VR10: list, download five files, delete."""
    info = require_rvc4()
    saved_env = hide_device_env()
    directory = tempfile.TemporaryDirectory(prefix="dai-py7-")
    download = tempfile.TemporaryDirectory(prefix="dai-py7-dl-")
    device = None
    pipeline = None
    name = f"py7_{os.getpid()}"
    try:
        device = dai.Device(info)
        if device.getPlatform() != dai.Platform.RVC4:
            pytest.skip("needs an RVC4 device")
        delete_recording(device, name)
        pipeline = dai.Pipeline(device)
        record = pipeline.create(dai.node.RecordVideo)
        record.setFormat(dai.RecordFormat.RAW_FRAMES)
        record.setRecordingName(name)
        record.setOutputDirectory(directory.name)
        record.setStreamName(STREAM)
        record.setTransferToHost(False)
        frames = record.input.createInputQueue()
        status = record.status.createOutputQueue(16, True)
        pipeline.start()
        assert_started(take_status(status, 30), 0)
        for n in range(5):
            # The .dai files stay on the device, so there is no host file count to poll.
            frames.send(make_record_frame(n))
            time.sleep(IDLE_SLEEP_S)
        pipeline.stop()
        pipeline.wait()
        stopped = take_status(status, 30)
        assert_stopped(stopped, 0, 5, 0, 4, 0)
        names = [recording.name for recording in device.listRecordings()]
        assert name in names
        device.downloadRecording(name, download.name)
        assert_dai_names(Path(download.name) / name / STREAM, range(5))
        device.deleteRecording(name)
        names = [recording.name for recording in device.listRecordings()]
        assert name not in names
    finally:
        stop_pipeline(pipeline)
        if device is not None:
            try:
                delete_recording(device, name)
            except Exception:
                pass
            device.close()
        download.cleanup()
        directory.cleanup()
        restore_env(saved_env)


_PY8_SCRIPT = """\
first = True
while True:
    node.inputs["go"].get()
    node.outputs["ctl"].send(RecordControl.start() if first else RecordControl.stop())
    first = not first
"""


@pytest.mark.rvc4
def test_py8_script_record_control():
    """PY8 mirrors MS-1: Script sends start then stop. STARTED(0), STOPPED(0, 3, 0, 2, 0), three files."""
    info = require_rvc4()
    saved_env = hide_device_env()
    directory = tempfile.TemporaryDirectory(prefix="dai-py8-")
    device = None
    pipeline = None
    name = f"py8_{os.getpid()}"
    try:
        device = dai.Device(info)
        if device.getPlatform() != dai.Platform.RVC4:
            pytest.skip("needs an RVC4 device")
        delete_recording(device, name)
        pipeline = dai.Pipeline(device)
        record = pipeline.create(dai.node.RecordVideo)
        record.setFormat(dai.RecordFormat.RAW_FRAMES)
        record.setRecordingName(name)
        record.setOutputDirectory(directory.name)
        record.setStreamName(STREAM)
        record.setWaitForTrigger(True)
        script = pipeline.create(dai.node.Script)
        go = script.inputs["go"].createInputQueue()
        script.setScript(_PY8_SCRIPT)
        script.outputs["ctl"].link(record.inputControl)
        frames = record.input.createInputQueue()
        status = record.status.createOutputQueue(16, True)
        pipeline.start()
        go.send(dai.Buffer())
        assert_started(take_status(status, 30), 0)
        for n in range(3):
            frames.send(make_record_frame(n))
            time.sleep(IDLE_SLEEP_S)
        go.send(dai.Buffer())
        assert_stopped(take_status(status, 30), 0, 3, 0, 2, 0)
        pipeline.stop()
        pipeline.wait()
        assert_dai_names(Path(directory.name) / name / STREAM, (0, 1, 2))
    finally:
        stop_pipeline(pipeline)
        if device is not None:
            try:
                delete_recording(device, name)
            except Exception:
                pass
            device.close()
        directory.cleanup()
        restore_env(saved_env)
