#!/usr/bin/env python3

"""
Minimal changes to original script:
  * Adds simple timestamp-based synchronisation across multiple devices.
  * Presents frames side‑by‑side when they are within 1 / FPS seconds.
  * Keeps v3 API usage and overall code structure intact.
"""

import contextlib
import datetime

import cv2
import depthai as dai
import time
import math

import argparse
# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SET_MANUAL_EXPOSURE = False  # Set to True to use manual exposure settings
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
        ).createOutputQueue(blocking=False)
    )
    if SET_MANUAL_EXPOSURE:
        camRgb.initialControl.setManualExposure(1000, 100)
    return pipeline, output


parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-d", "--devices", default=[], nargs="+", help="Device IPs")
parser.add_argument("-f", "--fps", type=float, default=30.0, help="Target FPS")
args = parser.parse_args()
DEVICE_INFOS = [dai.DeviceInfo(ip) for ip in args.devices] #The master camera needs to be first here
assert len(DEVICE_INFOS) > 1, "At least two devices are required for this example."

TARGET_FPS = args.fps  # Must match sensorFps in createPipeline()
SYNC_THRESHOLD_SEC = 1.0 / (2 * TARGET_FPS)  # Max drift to accept as "in sync"
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:

    queues = []
    slave_pipelines = []
    master_pipelines = []
    device_ids = []

    for idx, deviceInfo in enumerate(DEVICE_INFOS):
        pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
        device = pipeline.getDefaultDevice()

        print("=== Connected to", deviceInfo.getDeviceId())
        print("    Device ID:", device.getDeviceId())
        print("    Num of cameras:", len(device.getConnectedCameras()))

        # socket = device.getConnectedCameras()[0]
        socket = device.getConnectedCameras()[1]
        pipeline, out_q = createPipeline(pipeline, socket)
        role = device.getM8FsyncRole()
        if (role == dai.M8FsyncRole.MASTER):
            device.setM8StrobeEnable(True)
            device.setM8StrobeLimits(0.05, 0.95)
            master_pipelines.append(pipeline)
        elif (role == dai.M8FsyncRole.SLAVE):
            slave_pipelines.append(pipeline)
        else:
            raise RuntimeError(f"Don't know how to handle role {role}")
        
        queues.append(out_q)
        device_ids.append(deviceInfo.getXLinkDeviceDesc().name)
    
    if (len(master_pipelines) > 1):
        raise RuntimeError("Multiple masters detected!")
    
    if (len(master_pipelines) == 0):
        raise RuntimeError("No master detected!")
    
    if (len(slave_pipelines) < 1):
        raise RuntimeError("No slaves detected!")

    for p in master_pipelines:
        p.start()

    for p in slave_pipelines:
        p.start()


    # Buffer for latest frames; key = queue index
    latest_frames = {}
    fpsCounters = [FPSCounter() for _ in queues]
    receivedFrames = [False for _ in queues]
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
            # Build composite image side‑by‑side
            imgs = []
            for i in range(len(queues)):
                msg = latest_frames[i]
                frame = msg.getCvFrame()
                fps = fpsCounters[i].getFps()
                cv2.putText(
                    frame,
                    f"{device_ids[i]} | Timestamp: {ts_values[i]} | FPS:{fps:.2f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 50),
                    2,
                    cv2.LINE_AA,
                )
                imgs.append(frame)

            sync_status = "in sync" if abs(max(ts_values) - min(ts_values)) < 0.001 else "out of sync"
            delta = max(ts_values) - min(ts_values)
            color = (0, 255, 0) if sync_status == "in sync" else (0, 0, 255)
            
            cv2.putText(
                imgs[0],
                f"{sync_status} | delta = {delta*1e3:.3f} ms",
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("synced_view", cv2.hconcat(imgs))
            latest_frames.clear()  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
