"""Shared helpers for the record examples in this folder.

The scripts build a Pipeline and write under RR_OUTPUT (default rr-output).
Older firmware without the RecordVideo node is reported as a known limit.
Other errors are reported separately.
"""

import os
import time
from pathlib import Path

import depthai as dai

def output_dir():
    env = os.environ.get("RR_OUTPUT")
    if env:
        return Path(env)
    path = Path("rr-output")
    path.mkdir(parents=True, exist_ok=True)
    return path


def unique_name(prefix):
    return prefix + "_" + dai.makeRecordingTimestamp()


def status_name(state):
    if state == dai.RecordStatus.State.STARTED:
        return "STARTED"
    if state == dai.RecordStatus.State.STOPPED:
        return "STOPPED"
    if state == dai.RecordStatus.State.ERROR:
        return "ERROR"
    return "UNKNOWN"


def print_status(status):
    text = (
        f"status {status_name(status.state)} segment={status.segment} frames={status.frameCount}"
        f" seq={status.firstSequenceNum}..{status.lastSequenceNum} skipped={status.skippedFrames} path={status.path}"
    )
    if status.error:
        text += f" error={status.error}"
    print(text)


def classify_record_failure(text):
    print(f"EXCEPTION: {text}")
    firmware = "implemented by device firmware" in text
    # The device reports: Pipeline node with name: 'RecordVideo' doesn't exist.
    missing_node = "RecordVideo" in text and (
        "doesn't exist" in text
        or "does not exist" in text
        or "not found" in text
        or "unknown" in text
        or "Unknown" in text
        or "no node" in text
        or "not supported" in text
    )
    if firmware or missing_node:
        print("KNOWN_LIMIT: RecordVideo device execution is not in this firmware. The example still configured that path.")
        return 2
    return 1


def add_camera_nv12(pipeline, fps=10):
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    # 640x480 is a multiple of 32 by 8, which RVC2 encoding requires.
    return camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, fps)


def link_encoder(pipeline, frames, record, codec=dai.VideoEncoderProperties.Profile.H264_MAIN):
    encoder = pipeline.create(dai.node.VideoEncoder)
    # RecordVideo rejects H.265 in VIDEO format.
    encoder.setProfile(codec)
    frames.link(encoder.input)
    encoder.out.link(record.input)


def start_or_classify(pipeline):
    try:
        pipeline.start()
        return 0
    except Exception as ex:
        return classify_record_failure(str(ex))


def try_get(queue, datatype):
    message = queue.tryGet()
    if message is None:
        return None
    if not isinstance(message, datatype):
        return None
    return message


def record_for(pipeline, record, duration):
    print(f"RecordVideo runOnHost={record.runOnHost()}")
    status = record.status.createOutputQueue(16, False)
    try:
        pipeline.start()
    except Exception as ex:
        return classify_record_failure(str(ex))
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline and pipeline.isRunning():
        try:
            message = try_get(status, dai.RecordStatus)
            if message is not None:
                print_status(message)
        except Exception as ex:
            print(f"status poll: {ex}")
            break
        time.sleep(0.05)
    try:
        pipeline.stop()
    except Exception as ex:
        print(f"STOP_EXCEPTION: {ex}")
    last = record.getStatus()
    if last is not None:
        print("getStatus ", end="")
        print_status(last)
    else:
        print("getStatus: empty")
    # stop() closes the queue. A later tryGet throws "MessageQueue was closed".
    try:
        while True:
            message = try_get(status, dai.RecordStatus)
            if message is None:
                break
            print_status(message)
    except Exception as ex:
        print(f"status drain: {ex}")
    return 0


def finish_recording(pipeline, record, status):
    # Drain live queues before stop closes them; the final snapshot remains available
    # through getStatus(). Require an actual completed recording for these examples.
    completed = False
    failed = False

    def observe(message, show):
        nonlocal completed, failed
        if show:
            print_status(message)
        if message.state == dai.RecordStatus.State.STOPPED and message.frameCount > 0:
            completed = True
        if message.state == dai.RecordStatus.State.ERROR:
            failed = True

    while True:
        message = try_get(status, dai.RecordStatus)
        if message is None:
            break
        observe(message, True)
    pipeline.stop()
    last = record.getStatus()
    if last is not None:
        observe(last, False)
    if failed:
        raise RuntimeError("Recorder published ERROR")
    if not completed:
        raise RuntimeError("No completed segment with recorded frames")
