#!/usr/bin/env python3

"""
PTP example: fetch frames from devices and display them side by side.
Print time delta between frames on the displayed window.
"""

import contextlib
import datetime

import cv2
import depthai as dai
import time
# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
TARGET_FPS = 25  # Must match sensorFps in createPipeline()
SYNC_THRESHOLD_SEC = 1.0 / (2 * TARGET_FPS)  # Max drift to accept as "in sync"
SET_MANUAL_EXPOSURE = False  # Set to True to use manual exposure settings
# DEVICE_INFOS: list[dai.DeviceInfo] = ["IP_MASTER", "IP_SLAVE_1"] # Insert the device IPs here, e.g.:
DEVICE_INFOS = [dai.DeviceInfo(ip) for ip in ["192.168.0.146", "192.168.0.149"]] # The master camera needs to be first here
assert len(DEVICE_INFOS) > 1, "At least two devices are required for this example."
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


def formatTime(td: datetime.timedelta) -> str:
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
        camRgb.initialControl.setManualExposure(1000, 100)
    
    camRgb.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)

    return pipeline, output


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:
    # deviceInfos = dai.Device.getAllAvailableDevices()
    # print("=== Found devices: ", deviceInfos)

    queues = []
    pipelines = []
    deviceIds = []

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
        deviceIds.append(deviceInfo.getXLinkDeviceDesc().name)

    # Buffer for latest frames; key = queue index
    latestFrames = {}
    fpsCounters = [FPSCounter() for _ in queues]
    receivedFrames = [False for _ in queues]
    while True:
        # -------------------------------------------------------------------
        # Collect the newest frame from each queue (non‑blocking)
        # -------------------------------------------------------------------
        for idx, q in enumerate(queues):
            while q.has():
                latestFrames[idx] = q.get()
                if not receivedFrames[idx]:
                    print("=== Received frame from", deviceIds[idx])
                    receivedFrames[idx] = True
                fpsCounters[idx].tick()

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within SYNC_THRESHOLD_SEC.
        # -------------------------------------------------------------------
        if len(latestFrames) == len(queues):
            tsValues = [f.getTimestamp(dai.CameraExposureOffset.END).total_seconds() for f in latestFrames.values()]
            if max(tsValues) - min(tsValues) <= SYNC_THRESHOLD_SEC:
                # Build composite image side‑by‑side
                imgs = []
                for i in range(len(queues)):
                    msg = latestFrames[i]
                    frame = msg.getCvFrame()
                    fps = fpsCounters[i].getFps()
                    cv2.putText(
                        frame,
                        f"{deviceIds[i]} | Timestamp: {tsValues[i]} | FPS:{fps:.2f}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 0, 50),
                        2,
                        cv2.LINE_AA,
                    )
                    imgs.append(frame)

                syncStatus = "in sync" if abs(max(tsValues) - min(tsValues)) < 0.001 else "out of sync"
                delta = max(tsValues) - min(tsValues)
                color = (0, 255, 0) if syncStatus == "in sync" else (0, 0, 255)
                
                cv2.putText(
                    imgs[0],
                    f"{syncStatus} | delta = {delta*1e3:.3f} ms",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                    cv2.LINE_AA,
                )

                cv2.imshow("synced_view", cv2.hconcat(imgs))
                latestFrames.clear()  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
