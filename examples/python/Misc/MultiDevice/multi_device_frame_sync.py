#!/usr/bin/env python3

import contextlib
import datetime

import cv2
import depthai as dai
import time
import math

import argparse
import signal
import threading
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

from typing import Optional, Dict
from enum import Enum

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

class SyncType(Enum):
    EXTERNAL = 0
    PTP = 1

# ---------------------------------------------------------------------------
# Create camera outputs
# ---------------------------------------------------------------------------
def createCameraOutputs(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket, sensorFps: float, role: dai.ExternalFrameSyncRole):
    global syncType
    cam = None

    # Only specify FPS if camera is master
    if role == dai.ExternalFrameSyncRole.MASTER or syncType == SyncType.PTP:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket, sensorFps=sensorFps)
        )
    # Slave cameras will lock to the master's FPS
    else:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket)
        )

    output = (
        cam.requestOutput(
            (640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP
        )
    )

    if syncType == SyncType.PTP:
        cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
        print(f"Setting PTP for {socket.name}")

    cam.initialControl.setManualExposure(900, 1000)
    return pipeline, output

# ---------------------------------------------------------------------------
# Create synchronization node
# ---------------------------------------------------------------------------
def createSyncNode(syncThreshold: datetime.timedelta):
    global masterPipeline, masterNode, masterName, slaveQueues, inputQueues, outputNames, slavePipelines
    sync = masterPipeline.create(dai.node.Sync)
    #sync = masterPipeline.create(dai.node.SyncSystem)

    # Sync node will run on the host, since it needs to sync multiple devices
    sync.setRunOnHost(True)
    sync.setSyncThreshold(syncThreshold)

    # Link master camera outputs to the sync node
    for socketName, camOutput in masterNode.items():
        name = f"master_{masterName}_{socketName}"
        camOutput.link(sync.inputs[name])
        outputNames.append(name)

    # For slaves, we must create an input queue for each output
    # We will then manually forward the frames from each input queue to the output queue
    # This is because slave devices have separate pipelines from the master
    for deviceName, sockets in slaveQueues.items():
        for socketName, _ in sockets.items():
            name = f"slave_{deviceName}_{socketName}"
            outputNames.append(name)
            input_queue = sync.inputs[name].createInputQueue()
            inputQueues[name] = input_queue
    
    return sync

# ---------------------------------------------------------------------------
# Set up for individual camera sockets
# ---------------------------------------------------------------------------
def setUpCameraSocket(
        pipeline: dai.Pipeline,
        socket: dai.CameraBoardSocket,
        deviceName: str,
        targetFps: float,
        role: dai.ExternalFrameSyncRole):
    global masterNode, slaveQueues, camSockets, syncType, ptpMasterDeviceName
    pipeline, outNode = createCameraOutputs(pipeline, socket, targetFps, role)

    if syncType == SyncType.EXTERNAL:
        # Master cameras will be linked to the sync node directly
        if role == dai.ExternalFrameSyncRole.MASTER:
            if masterNode is None:
                masterNode = {}
            
            masterNode[socket.name] = outNode
        
        # Gather all slave camera outputs
        elif role == dai.ExternalFrameSyncRole.SLAVE:
            if slaveQueues.get(deviceName) is None:
                slaveQueues[deviceName] = {}
            slaveQueues[deviceName][socket.name] = outNode.createOutputQueue()
        else:
            raise RuntimeError(f"Don't know how to handle role {role}")
    elif syncType == SyncType.PTP:
        # For PTP just put the first camera in master
        # Actual PTP master might be different, but it doesn't matter for this example
        if masterNode is None:
            masterNode = {}
            ptpMasterDeviceName = deviceName
        
        if ptpMasterDeviceName == deviceName:
            masterNode[socket.name] = outNode
        else:
            if slaveQueues.get(deviceName) is None:
                slaveQueues[deviceName] = {}
            slaveQueues[deviceName][socket.name] = outNode.createOutputQueue()

    
    # Keep track of all camera socket names
    if socket.name not in camSockets:
        camSockets.append(socket.name)

    return pipeline

def getDeviceName(device : dai.Device) -> str:
    info = device.getDeviceInfo()
    name = info.deviceId
    if info.name is not None and info.name != "":
        name += "[" + info.name + "]"
    return name

def setupDevice(
        stack: contextlib.ExitStack,
        deviceInfo: dai.DeviceInfo,
        targetFps: float):
    global masterPipeline, masterName, slavePipelines, slaveQueues, camSockets, syncType

    # Create pipeline for device
    pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
    device = pipeline.getDefaultDevice()

    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError("This example supports only RVC4 platform!")

    name = getDeviceName(device)
    role = None
    if syncType == SyncType.EXTERNAL:
        role = device.getExternalFrameSyncRole()

    print("=== Connected to", deviceInfo.getDeviceId())
    print("    Device ID:", device.getDeviceId())
    print("    Num of cameras:", len(device.getConnectedCameras()))

    sensorNames = device.getCameraSensorNames()
    for socket in device.getConnectedCameras():
        sensorName = ""
        for sckt, namee in sensorNames.items():
            if sckt == socket:
                sensorName = namee
                break
        if sensorName == "":
            print(f"Could not find sensor name for socket {socket} for device {name}")
            continue
        #if "IMX" in sensorName:
        #    print(f"Skipping IMX sensor {sensorName} for device {name}")
        #    continue
        #if "OV" in sensorName:
        #    print(f"Skipping OV sensor {sensorName} for device {name}")
        #    continue
        # if "OG" in sensorName:
        #     print(f"Skipping OG sensor {sensorName} for device {name}")
        #     continue
        #if socket == dai.CameraBoardSocket.CAM_A:
        #    print(f"Skipping CAM_B sensor {sensorName} for device {name}")
        #    continue
        print(f"Setting up socket {socket} for device {name}")
        pipeline = setUpCameraSocket(pipeline, socket, name, targetFps, role)

    if syncType == SyncType.EXTERNAL:
        if role == dai.ExternalFrameSyncRole.MASTER:
            device.setExternalStrobeEnable(True)
            print(f"{device.getDeviceId()} is master")

            if masterPipeline is not None:
                raise RuntimeError("Only one master pipeline is supported")
            
            masterPipeline = pipeline
            masterName = name
        elif role == dai.ExternalFrameSyncRole.SLAVE:
            slavePipelines[name] = pipeline
            print(f"{device.getDeviceId()} is slave")
        else:
            raise RuntimeError(f"Don't know how to handle role {role}")
    elif syncType == SyncType.PTP:
        # For PTP just put the first camera in master
        # Actual PTP master might be different, but it doesn't matter for this example
        if masterPipeline is None:
            masterPipeline = pipeline
            masterName = name
        else:
            slavePipelines[name] = pipeline


running: threading.Event = threading.Event()
running.set()

def interruptHandler(sig, frame):
    global running
    if running.is_set:
        print("Interrupted! Exiting...")
        running.clear()
    else:
        print("Exiting now!")
        exit(0)

signal.signal(signal.SIGINT, interruptHandler)

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("-f", "--fps", type=float, default=30.0, help="Target FPS", required=False)
parser.add_argument("-d", "--devices", default=[], nargs="+", help="Device IPs or IDs", required=False)
parser.add_argument("-t1", "--recv-all-timeout-sec", type=float, default=10, help="Timeout for receiving the first frame from all devices", required=False)
parser.add_argument("-t2", "--sync-threshold-sec", type=float, default=1e-3, help="Sync threshold in seconds", required=False)
parser.add_argument("-t3", "--initial-sync-timeout-sec", type=float, default=4, help="Timeout for synchronization to complete", required=False)
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--external-sync", action="store_true", help="Use external sync")
group.add_argument("--ptp-sync", action="store_true", help="Use PTP sync")
args = parser.parse_args()

# if user did not specify device IPs, use all available devices
if len(args.devices) == 0:
    deviceInfos = dai.Device.getAllAvailableDevices()
else:
    deviceInfos = [dai.DeviceInfo(ip) for ip in args.devices]

assert len(deviceInfos) > 1, "At least two devices are required for this example."

targetFps = args.fps  # Must match sensorFps in createPipeline()
recvAllTimeoutSec = args.recv_all_timeout_sec

syncThresholdSec = args.sync_threshold_sec
initialSyncTimeoutSec = args.initial_sync_timeout_sec
if args.external_sync:
    syncType = SyncType.EXTERNAL
elif args.ptp_sync:
    syncType = SyncType.PTP
    print("Master camera does not match PTP master, instead it signifies the sync pipeline")
else:
    raise RuntimeError("Must specify sync type")
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:

    # Variables to keep track of master and slave pipelines and outputs
    masterPipeline: Optional[dai.Pipeline] = None
    masterNode: Optional[Dict[str, dai.Node.Output]] = None
    masterName: Optional[str] = None

    slavePipelines: Dict[str, dai.Pipeline] = {}
    slaveQueues: Dict[str, Dict[str, dai.MessageQueue]] = {}

    # keep track of sync node inputs for slaves
    inputQueues = {}

    # keep track of all sync node output names
    outputNames = []

    # keep track of all camera socket names
    camSockets = []

    ptpMasterDeviceName = ""

    for idx, deviceInfo in enumerate(deviceInfos):
        setupDevice(stack, deviceInfo, targetFps)

    if masterPipeline is None or masterNode is None:
        raise RuntimeError("No master detected!")

    if len(slavePipelines) < 1:
        raise RuntimeError("No slaves detected!")

    # Create sync node
    # Sync node groups the frames so that all synced frames are timestamped to within one frame time
    sync = createSyncNode(datetime.timedelta(milliseconds=1000 / (2 * targetFps)))
    queue = sync.out.createOutputQueue()

    masterPipeline.start()
    for _, slave_cam_socket in slavePipelines.items():
        slave_cam_socket.start()

    fpsCounter = FPSCounter()

    latestFrameGroup = None
    firstReceived = False
    startTime = datetime.datetime.now()
    prevReceived = datetime.datetime.now()

    initialSyncTime = None
    waitingForSync = True

    allDeltas = []
    refTimestamps = []
    tmpDeltas = []
    num_tmp_deltas = 1500
    num_deltas_to_sample = 30
    steady_state_criterion = 0.05e-3
    steady_state_reached_index = -1
    steady_state_timeout = 10 * 60

    def data_collector(deviceName, socketName):
        # Send frames from slave output queues to sync node input queues
        camOutputQueue = slaveQueues[deviceName][socketName]
        while running.is_set():
            try:
                if camOutputQueue.has():
                    inputQueues[f"slave_{deviceName}_{socketName}"].send(camOutputQueue.get())
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"Exception in data_collector {deviceName} {socketName}: {e}")
                running.clear()

    threads = {}

    for deviceName, sockets in slaveQueues.items():
        for socketName, camOutputQueue in sockets.items():
            threads[f"slave_{deviceName}_{socketName}"] = threading.Thread(target=data_collector, args=(deviceName, socketName))
            threads[f"slave_{deviceName}_{socketName}"].start()

    while running.is_set():
        # Get frames from sync node output queue
        while queue.has():
            latestFrameGroup = queue.get()
            if not firstReceived:
                firstReceived = True
                initialSyncTime = datetime.datetime.now()
            prevReceived = datetime.datetime.now()
            fpsCounter.tick()
        
        # Timeout if we dont receive any frames at the beginning
        if not firstReceived:
            endTime = datetime.datetime.now()
            elapsedSec = (endTime - startTime).total_seconds()
            if elapsedSec >= recvAllTimeoutSec:
                print(f"Timeout: Didn't receive all frames in time: {elapsedSec:.2f} sec")
                running.clear()

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within syncThresholdSec.
        # -------------------------------------------------------------------
        if latestFrameGroup is not None and latestFrameGroup.getNumMessages() == len(outputNames):
            tsValues = {}
            for name in outputNames:
                #tsValues[name] = (latestFrameGroup[name].getTimestampSystem(dai.CameraExposureOffset.END) - datetime.datetime.fromtimestamp(0)).total_seconds()
                tsValues[name] = latestFrameGroup[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds()
            
            # Build individual image arrays for each camera socket, displayed side-by-side
            imgs = []
            for name in camSockets:
                imgs.append([])
            fps = fpsCounter.getFps()

            # calculate the greatest time difference between all frames
            delta = max(tsValues.values()) - min(tsValues.values())
            refTs = min(tsValues.values())

            syncStatus = abs(delta) < syncThresholdSec
            syncStatusStr = "in sync" if syncStatus else "out of sync"

            # Timeout if frames don't get synced in time
            if not syncStatus and waitingForSync:
                endTime = datetime.datetime.now()
                elapsedSec = (endTime - initialSyncTime).total_seconds()
                if elapsedSec >= initialSyncTimeoutSec:
                    print("Timeout: Didn't sync frames in time")
                    running.clear()

            if syncStatus and waitingForSync:
                print(f"Frame synced")
                waitingForSync = False

            if not syncStatus and not waitingForSync:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                continue

            tmpDeltas.append(delta)
            if steady_state_reached_index < 0:
                if len(tmpDeltas) > num_tmp_deltas:
                    oldest = tmpDeltas.pop(0)
                    samples = random.sample(tmpDeltas, num_deltas_to_sample)
                    t1 = min([min(samples), delta])
                    t2 = max([max(samples), oldest])

                    if abs(t2 - t1) < steady_state_criterion:
                        print(f"Steady state reached at {len(allDeltas)}, delta: {delta * 1e6} us")
                        steady_state_reached_index = len(allDeltas)
                        steady_state_start_time = datetime.datetime.now()

                endTime = datetime.datetime.now()
                elapsedSec = (endTime - startTime).total_seconds()
                if elapsedSec > steady_state_timeout:
                    print(f"PTP did not reach steady state after {elapsedSec} seconds")
                    running.clear()
            
            allDeltas.append(delta)
            refTimestamps.append(refTs)

            color = (0, 255, 0) if syncStatusStr == "in sync" else (0, 0, 255)

            # Create a image frame with sync info for each output
            for outputName in outputNames:
                # Find out which camera socket this output belongs to
                idx = -1
                for i, name in enumerate(camSockets):
                    if name in outputName:
                        idx = i
                        break
                if idx == -1:
                    raise RuntimeError(f"Could not find camera socket for {outputName}")
                
                # Get frame for this output
                msg = latestFrameGroup[outputName]
                frame = msg.getCvFrame()

                # Add output name to frame
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

                # Add timestamp and FPS to frame
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

            # Add absolute maximum time difference between all frames
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

            # Show the frame
            for i, img in enumerate(imgs):
                cv2.imshow(f"synced_view_{camSockets[i]}", cv2.hconcat(imgs[i]))

            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            running.clear()
            break

    for t in threads.keys():
        threads[t].join()

    cv2.destroyAllWindows()

    if steady_state_reached_index > 0:
        xAxisTimestamps = [t - refTimestamps[steady_state_reached_index] for t in refTimestamps]
        prevTs = None
        numFrames = len(allDeltas[steady_state_reached_index:])
        lineCutoff = 0
        definitelyLostFrames = 0
        possibleLostFrames = 0
        for i, ts in enumerate(xAxisTimestamps):
            if prevTs is not None:
                if i < steady_state_reached_index:
                    prevTs = ts
                    continue
                elif i == steady_state_reached_index:
                    lineCutoff = ts
                
                if ts - prevTs > 2/targetFps:
                    definitelyLostFrames += math.floor((ts - prevTs) * targetFps - 1)
                elif ts - prevTs > 1.5/targetFps:
                    possibleLostFrames += 1
            prevTs = ts

        meanDelta = np.mean(allDeltas[steady_state_reached_index:])
        p99Delta = np.percentile(allDeltas[steady_state_reached_index:], 99)
        maxDelta = np.max(allDeltas[steady_state_reached_index:])
        print(f"PTP reached steady state after {steady_state_reached_index} frames\n\tmean delta: {meanDelta * 1e3} ms\n\tp99 delta: {p99Delta * 1e3} ms\n\tmax delta: {maxDelta * 1e3} ms")
        print(f"Possible lost frames: {possibleLostFrames}, definitely lost frames: {definitelyLostFrames}")

        fig, ax = plt.subplots()
        ax.scatter(xAxisTimestamps, allDeltas, marker="X", s=5)
        tmp = max(allDeltas)
        line = mlines.Line2D([lineCutoff, lineCutoff], [0, tmp], color='r')
        ax.add_line(line)
        plt.ylabel("Delta (s)")
        plt.xlabel("time (s)")
        plt.title("PTP deltas")
        plt.show()

        # plot histogram of deltas after reaching steady state
        plt.hist(allDeltas[steady_state_reached_index:], bins=100)
        plt.show()
    else:
        print("PTP did not reach steady state")
