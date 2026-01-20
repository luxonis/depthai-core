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
import signal
import numpy as np
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


# ---------------------------------------------------------------------------
# Pipeline creation (unchanged API – only uses TARGET_FPS constant)
# ---------------------------------------------------------------------------
def createPipeline(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket, sensorFps: int, role: dai.M8FsyncRole):
    cam = None
    if role == dai.M8FsyncRole.MASTER:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket, sensorFps=sensorFps)
        )
    else:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket)
        )

    output = (
        cam.requestOutput(
            (640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH
        )
    )
    return pipeline, output

running = True

def interruptHandler(sig, frame):
    global running
    if running:
        print("Interrupted! Exiting...")
        running = False
    else:
        print("Exiting now!")
        exit(0)

signal.signal(signal.SIGINT, interruptHandler)

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-f", "--fps", type=float, default=30.0, help="Target FPS", required=False)
parser.add_argument("-d", "--devices", default=[], nargs="+", help="Device IPs", required=False)
parser.add_argument("-t1", "--recv-all-timeout-sec", type=float, default=10, help="Timeout for receiving the first frame from all devices", required=False)
parser.add_argument("-t2", "--sync-threshold-sec", type=float, default=3e-3, help="Sync threshold in seconds", required=False)
parser.add_argument("-t3", "--initial-sync-timeout-sec", type=float, default=4, help="Timeout for synchronization to complete", required=False)
args = parser.parse_args()

if len(args.devices) == 0:
    deviceInfos = dai.Device.getAllAvailableDevices()
else:
    deviceInfos = [dai.DeviceInfo(ip) for ip in args.devices] #The master camera needs to be first here

assert len(deviceInfos) > 1, "At least two devices are required for this example."

targetFps = args.fps  # Must match sensorFps in createPipeline()
recvAllTimeoutSec = args.recv_all_timeout_sec

syncThresholdSec = args.sync_threshold_sec
initialSyncTimeoutSec = args.initial_sync_timeout_sec
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:

    masterNodes = []
    slaveQueues = []
    slavePipelines = []
    masterPipelines = []
    deviceIds = []

    inputQueues = []
    slaveInputNames = []
    outputNames = []

    for idx, deviceInfo in enumerate(deviceInfos):
        pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
        device = pipeline.getDefaultDevice()

        role = device.getM8FsyncRole()

        print("=== Connected to", deviceInfo.getDeviceId())
        print("    Device ID:", device.getDeviceId())
        print("    Num of cameras:", len(device.getConnectedCameras()))

        socket = device.getConnectedCameras()[0]
        if "OAK4-D" in device.getProductName() or \
            "OAK-4-D" in device.getProductName():
            socket = device.getConnectedCameras()[1]

        pipeline, outNode = createPipeline(pipeline, socket, targetFps, role)

        if role == dai.M8FsyncRole.MASTER:
            device.setM8StrobeEnable(True)
            device.setM8StrobeLimits(0.05, 0.95)
            print(f"{device.getDeviceId()} is master")
            masterPipelines.append(pipeline)
            masterNodes.append(outNode)
        elif role == dai.M8FsyncRole.SLAVE:
            slavePipelines.append(pipeline)
            slaveQueues.append(outNode.createOutputQueue())
            print(f"{device.getDeviceId()} is slave")
        else:
            raise RuntimeError(f"Don't know how to handle role {role}")

        deviceIds.append(deviceInfo.getXLinkDeviceDesc().name)

    if len(masterPipelines) > 1:
        raise RuntimeError("Multiple masters detected!")

    if len(masterPipelines) == 0:
        raise RuntimeError("No master detected!")

    if len(slavePipelines) < 1:
        raise RuntimeError("No slaves detected!")

    sync = masterPipelines[0].create(dai.node.Sync)
    sync.setRunOnHost(True)
    sync.setSyncThreshold(datetime.timedelta(milliseconds=1000 / (2 * targetFps)))
    masterNodes[0].link(sync.inputs["master"])
    outputNames.append("master")

    for i in range(len(slaveQueues)):
        name = f"slave_{i}"
        slaveInputNames.append(name)
        outputNames.append(name)
        input_queue = sync.inputs[name].createInputQueue()
        inputQueues.append(input_queue)

    queue = sync.out.createOutputQueue()

    for p in masterPipelines:
        p.start()

    for p in slavePipelines:
        p.start()


    fpsCounter = FPSCounter()

    latestFrameGroup = None
    firstReceived = False
    startTime = datetime.datetime.now()
    prevReceived = datetime.datetime.now()

    initialSyncTime = None
    waitingForSync = True

    while running:

        for i, slq in enumerate(slaveQueues):
            while slq.has():
                inputQueues[i].send(slq.get())

        while queue.has():
            latestFrameGroup = queue.get()
            if not firstReceived:
                firstReceived = True
                initialSyncTime = datetime.datetime.now()
            prevReceived = datetime.datetime.now()
            fpsCounter.tick()

        if not firstReceived:
            endTime = datetime.datetime.now()
            elapsedSec = (endTime - startTime).total_seconds()
            if elapsedSec >= recvAllTimeoutSec:
                print(f"Timeout: Didn't receive all frames in time: {elapsedSec:.2f} sec")
                running = False

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within SYNC_THRESHOLD_SEC.
        # -------------------------------------------------------------------
        if latestFrameGroup is not None and latestFrameGroup.getNumMessages() == len(outputNames):
            tsValues = [latestFrameGroup[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds() for name in outputNames]
            # Build composite image side‑by‑side
            imgs = []
            fps = fpsCounter.getFps()

            for i, outputName in enumerate(outputNames):
                msg = latestFrameGroup[outputName]
                frame = msg.getCvFrame()
                cv2.putText(
                    frame,
                    f"{deviceIds[i]} ({outputName})",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 127, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"Timestamp: {tsValues[i]} | FPS:{fps:.2f}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 50),
                    2,
                    cv2.LINE_AA,
                )
                imgs.append(frame)

            delta = max(tsValues) - min(tsValues)

            syncStatus = abs(delta) < syncThresholdSec
            syncStatusStr = "in sync" if syncStatus else "out of sync"

            if not syncStatus and waitingForSync:
                endTime = datetime.datetime.now()
                elapsedSec = (endTime - initialSyncTime).total_seconds()
                if elapsedSec >= initialSyncTimeoutSec:
                    print("Timeout: Didn't sync frames in time")
                    running = False

            if syncStatus and waitingForSync:
                print(f"Sync status: {syncStatusStr}")
                waitingForSync = False

            if not syncStatus and not waitingForSync:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                running = False

            color = (0, 255, 0) if syncStatusStr == "in sync" else (0, 0, 255)
            
            cv2.putText(
                imgs[0],
                f"{syncStatusStr} | delta = {delta*1e3:.3f} ms",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("synced_view", cv2.hconcat(imgs))

            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
