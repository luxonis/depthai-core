#!/usr/bin/env python3

from collections import deque

import depthai as dai


FPS = 20.0
FRAME_PERIOD_MS = 1000.0 / FPS
FIRST_SOCKET = dai.CameraBoardSocket.CAM_A
SECOND_SOCKET = dai.CameraBoardSocket.CAM_D
FIRST_ID = 0
SECOND_ID = 1
DELAY_US = 0
BOARD_FSYNC_GPIO = 40


def timestamp_ms(frame: dai.ImgFrame) -> float:
    return frame.getTimestampDevice().total_seconds() * 1000.0


def drain_into(queue, buffer):
    while True:
        frame = queue.tryGet()
        if frame is None:
            return
        buffer.append(frame)


def wrap_delta_ms(delta_ms: float) -> float:
    return ((delta_ms + (FRAME_PERIOD_MS * 0.5)) % FRAME_PERIOD_MS) - (FRAME_PERIOD_MS * 0.5)


print("OAK-FFC-4P R5 internal trigger test")
print(f"GPIO={BOARD_FSYNC_GPIO} FPS={FPS} A(id={FIRST_ID}) D(id={SECOND_ID})")

device_config = dai.Device.Config()
device_config.board.ffcFsyncGeneratorGpio = BOARD_FSYNC_GPIO

with dai.Device(device_config) as device:
    pipeline = dai.Pipeline(device)

    first_tof = pipeline.create(dai.node.ToF).build(FIRST_SOCKET, dai.ImageFiltersPresetMode.TOF_MID_RANGE, fps=FPS)
    second_tof = pipeline.create(dai.node.ToF).build(SECOND_SOCKET, dai.ImageFiltersPresetMode.TOF_MID_RANGE, fps=FPS)

    first_tof.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    first_tof.initialControl.setFrameSyncId(FIRST_ID)
    first_tof.initialControl.setFrameSyncDelayUs(DELAY_US)

    second_tof.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    second_tof.initialControl.setFrameSyncId(SECOND_ID)
    second_tof.initialControl.setFrameSyncDelayUs(DELAY_US)

    first_queue = first_tof.depth.createOutputQueue(maxSize=1, blocking=False)
    second_queue = second_tof.depth.createOutputQueue(maxSize=1, blocking=False)

    pipeline.start()

    first_buffer = deque()
    second_buffer = deque()
 
    while pipeline.isRunning():
        drain_into(first_queue, first_buffer)
        drain_into(second_queue, second_buffer)

        while first_buffer and second_buffer:
            first = first_buffer.popleft()
            second = second_buffer.popleft()

            first_ts_ms = timestamp_ms(first)
            second_ts_ms = timestamp_ms(second)
            delta_ms = second_ts_ms - first_ts_ms
            delta_wrapped_ms = wrap_delta_ms(delta_ms)
            seq_delta = second.getSequenceNum() - first.getSequenceNum()

            print(
                f"A seq={first.getSequenceNum()} ts_ms={first_ts_ms:.3f} | "
                f"D seq={second.getSequenceNum()} ts_ms={second_ts_ms:.3f} | "
                f"seq_delta={seq_delta:+d} | "
                f"delta_ms={delta_ms:.3f} | "
                f"delta_wrapped_ms={delta_wrapped_ms:.3f}"
            )
