#!/usr/bin/env python3

import contextlib
import datetime

import cv2
import depthai as dai
import time
import math

import argparse
import signal
import numpy as np

from typing import Optional, Dict, List
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
def createPipeline(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket, sensorFps: float, role: dai.ExternalFrameSyncRole):
    cam = None
    if role == dai.ExternalFrameSyncRole.MASTER:
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

def createSyncNode(syncThreshold: datetime.timedelta):
    global masterPipeline, masterNode, slaveQueues, inputQueues, outputNames
    sync = masterPipeline.create(dai.node.Sync)
    sync.setRunOnHost(True)
    sync.setSyncThreshold(syncThreshold)
    for k, v in masterNode.items():
        name = f"master_{k}"
        v.link(sync.inputs[name])
        outputNames.append(name)

    for k, v in slaveQueues.items():
        for k2, v2 in v.items():
            name = f"slave_{k}_{k2}"
            outputNames.append(name)
            input_queue = sync.inputs[name].createInputQueue()
            inputQueues[name] = input_queue
    
    return sync

def setUpCameraSocket(
        pipeline: dai.Pipeline,
        socket: dai.CameraBoardSocket,
        name: str,
        targetFps: float,
        role: dai.ExternalFrameSyncRole):
    global masterNode, slaveQueues, camSockets
    pipeline, outNode = createPipeline(pipeline, socket, targetFps, role)

    if role == dai.ExternalFrameSyncRole.MASTER:
        if masterNode is None:
            masterNode = {}
        
        masterNode[socket.name] = outNode
    elif role == dai.ExternalFrameSyncRole.SLAVE:
        if slaveQueues.get(name) is None:
            slaveQueues[name] = {}
        
        slaveQueues[name][socket.name] = outNode.createOutputQueue()
    else:
        raise RuntimeError(f"Don't know how to handle role {role}")
    
    if socket.name not in camSockets:
        camSockets.append(socket.name)

    return pipeline

def setupDevice(
        stack: contextlib.ExitStack,
        deviceInfo: dai.DeviceInfo,
        sockets: Optional[List[str]],
        targetFps: float):
    global masterPipeline, masterNode, slavePipelines, slaveQueues, camSockets
    pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
    device = pipeline.getDefaultDevice()
    name = deviceInfo.getXLinkDeviceDesc().name

    role = device.getExternalFrameSyncRole()

    print("=== Connected to", deviceInfo.getDeviceId())
    print("    Device ID:", device.getDeviceId())
    print("    Num of cameras:", len(device.getConnectedCameras()))

    for socket in device.getConnectedCameras():
        if sockets is not None and socket.name not in sockets:
            continue
        pipeline = setUpCameraSocket(pipeline, socket, name, targetFps, role)

    if role == dai.ExternalFrameSyncRole.MASTER:
        device.setExternalStrobeEnable(True)
        print(f"{device.getDeviceId()} is master")

        if masterPipeline is not None:
            raise RuntimeError("Only one master pipeline is supported")
        
        masterPipeline = pipeline
    elif role == dai.ExternalFrameSyncRole.SLAVE:
        slavePipelines[name] = pipeline
        print(f"{device.getDeviceId()} is slave")
    else:
        raise RuntimeError(f"Don't know how to handle role {role}")


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
parser.add_argument("--sockets", type=str, help="Socket ids to sync, comma separated", required=False)
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

if args.sockets:
    sockets = args.sockets.split(",")
else:
    sockets = None

recvAllTimeoutSec = args.recv_all_timeout_sec

syncThresholdSec = args.sync_threshold_sec
initialSyncTimeoutSec = args.initial_sync_timeout_sec
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:

    masterPipeline: Optional[dai.Pipeline] = None
    masterNode: Optional[Dict[str, dai.Node.Output]] = None

    slavePipelines: Dict[str, dai.Pipeline] = {}
    slaveQueues: Dict[str, Dict[str, dai.MessageQueue]] = {}

    inputQueues = {}

    outputNames = []

    camSockets = []

    for idx, deviceInfo in enumerate(deviceInfos):
        setupDevice(stack, deviceInfo, sockets, targetFps)

    if masterPipeline is None or masterNode is None:
        raise RuntimeError("No master detected!")

    if len(slavePipelines) < 1:
        raise RuntimeError("No slaves detected!")

    sync = createSyncNode(datetime.timedelta(milliseconds=1000 / (2 * targetFps)))
    queue = sync.out.createOutputQueue()

    masterPipeline.start()
    for k, v in slavePipelines.items():
        v.start()

    fpsCounter = FPSCounter()

    latestFrameGroup = None
    firstReceived = False
    startTime = datetime.datetime.now()
    prevReceived = datetime.datetime.now()

    initialSyncTime = None
    waitingForSync = True

    while running:

        for k, v in slaveQueues.items():
            for k2, v2 in v.items():
                while v2.has():
                    inputQueues[f"slave_{k}_{k2}"].send(v2.get())

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
            tsValues = {}
            for name in outputNames:
                tsValues[name] = latestFrameGroup[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds()
            # Build composite image side‑by‑side
            imgs = []
            for name in camSockets:
                imgs.append([])
            fps = fpsCounter.getFps()

            for outputName in outputNames:
                idx = -1
                for i, name in enumerate(camSockets):
                    if name in outputName:
                        idx = i
                        break
                if idx == -1:
                    raise RuntimeError(f"Could not find camera socket for {outputName}")
                
                msg = latestFrameGroup[outputName]
                frame = msg.getCvFrame()
                cv2.putText(
                    frame,
                    f"{outputName}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 127, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"Timestamp: {tsValues[outputName]} | FPS:{fps:.2f}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 50),
                    2,
                    cv2.LINE_AA,
                )
                imgs[idx].append(frame)

            delta = max(tsValues.values()) - min(tsValues.values())

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
            
            for i, img in enumerate(imgs):
                cv2.putText(
                    imgs[i][0],
                    f"{syncStatusStr} | delta = {delta*1e3:.3f} ms",
                    (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                    cv2.LINE_AA,
                )

            for i, img in enumerate(imgs):
                cv2.imshow(f"synced_view_{camSockets[i]}", cv2.hconcat(imgs[i]))

            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
