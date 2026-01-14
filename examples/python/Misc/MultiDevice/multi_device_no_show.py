#!/usr/bin/env python3

"""
PTP example: fetch frames from devices and log time delta between frames
to FRAME_DELTA_LOG_FILE.
"""

import contextlib
import datetime

import depthai as dai
import time
import csv
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
TARGET_FPS = 25  # Must match sensorFps in createPipeline()
SYNC_THRESHOLD_SEC = 1.0 / (2 * TARGET_FPS)  # Max drift to accept as "in sync"
SET_MANUAL_EXPOSURE = True  # Set to True to use manual exposure settings
# DEVICE_INFOS: list[dai.DeviceInfo] = ["IP_MASTER", "IP_SLAVE_1"] # Insert the device IPs here, e.g.:
DEVICE_INFOS = [dai.DeviceInfo(ip) for ip in ["192.168.0.124", "192.168.0.122"]] # The master camera needs to be first here
assert len(DEVICE_INFOS) > 1, "At least two devices are required for this example."

start_ts = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
FRAME_DELTA_LOG_FILE = Path(f"/tmp/frame_deltas_{start_ts}.csv")
print(f"Saving frame log file to {FRAME_DELTA_LOG_FILE}")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-100:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        # Calculate the FPS
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])


def format_time(td: datetime.timedelta) -> str:
    hours, remainder_seconds = divmod(td.seconds, 3600)
    minutes, seconds = divmod(remainder_seconds, 60)
    milliseconds, microseconds_remainder = divmod(td.microseconds, 1000)
    days_prefix = f"{td.days} day{'s' if td.days != 1 else ''}, " if td.days else ""
    return (
        f"{days_prefix}{hours:02d}:{minutes:02d}:{seconds:02d}."
        f"{milliseconds:03d}.{microseconds_remainder:03d}"
    )


# ---------------------------------------------------------------------------
# Pipeline creation (unchanged API – only uses TARGET_FPS constant)
# ---------------------------------------------------------------------------
def createPipeline(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket = dai.CameraBoardSocket.CAM_A):
    camRgb = (
        pipeline.create(dai.node.Camera)
        .build(socket, sensorFps=TARGET_FPS)
    )
    output = (
        camRgb.requestOutput(
            (640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH
        ).createOutputQueue()
    )
    if SET_MANUAL_EXPOSURE:
        camRgb.initialControl.setManualExposure(2000, 400)
    return pipeline, output


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:
    # deviceInfos = dai.Device.getAllAvailableDevices()
    # print("=== Found devices: ", deviceInfos)

    queues = []
    pipelines = []
    device_ids = []

    for deviceInfo in DEVICE_INFOS:
        pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
        device = pipeline.getDefaultDevice()

        print("=== Connected to", deviceInfo.getDeviceId())
        print("    Device ID:", device.getDeviceId())
        print("    Num of cameras:", len(device.getConnectedCameras()))

        socket = device.getConnectedCameras()[0]
        pipeline, out_q = createPipeline(pipeline, socket)
        pipeline.start()

        pipelines.append(pipeline)
        queues.append(out_q)
        device_ids.append(deviceInfo.getXLinkDeviceDesc().name)

    # Buffer for latest frames; key = queue index
    latest_frames = {}
    fpsCounters = [FPSCounter() for _ in queues]
    receivedFrames = [False for _ in queues]
    delta_write_buffer = []
    frame_idx = 0

    # Create file for saving frame deltas
    try:
        with open(FRAME_DELTA_LOG_FILE, "r", newline="") as f:
            has_data = f.read(1) != ""
    except FileNotFoundError:
        has_data = False

    if not has_data:
        with open(FRAME_DELTA_LOG_FILE, "w", newline="") as f:
            csv.writer(f).writerow(["frame_idx", "delta_t [s]"])

    while True:
        # -------------------------------------------------------------------
        # Collect the newest frame from each queue (non‑blocking)
        # -------------------------------------------------------------------
        for idx, q in enumerate(queues):
            while q.has():
                latest_frames[idx] = q.get()
                if not receivedFrames[idx]:
                    print("=== Received frame from", device_ids[idx])
                    receivedFrames[idx] = True
                fpsCounters[idx].tick()

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within SYNC_THRESHOLD_SEC.
        # -------------------------------------------------------------------
        if len(latest_frames) == len(queues):
            ts_values = [f.getTimestamp(dai.CameraExposureOffset.END).total_seconds() for f in latest_frames.values()]
            if max(ts_values) - min(ts_values) <= SYNC_THRESHOLD_SEC:
                # Build composite image side‑by‑side
                imgs = []
                for i in range(len(queues)):
                    msg = latest_frames[i]
                    frame = msg.getCvFrame()
                    fps = fpsCounters[i].getFps()

                delta = max(ts_values) - min(ts_values)
                print(f"Frame delta [ms]: {delta*1e3:.3f}")

                # Frame delta logging every N frames
                delta_write_buffer.append((frame_idx, delta))
                frame_idx += 1

                if len(delta_write_buffer) >= 30:
                    with open(FRAME_DELTA_LOG_FILE, "a", newline="") as f:
                        print("saving")
                        writer = csv.writer(f)
                        writer.writerows(delta_write_buffer)
                    delta_write_buffer.clear()
                
                latest_frames.clear()  # Wait for next batch
