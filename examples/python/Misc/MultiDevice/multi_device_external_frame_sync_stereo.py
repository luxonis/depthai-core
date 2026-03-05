#!/usr/bin/env python3

import contextlib
import datetime

import cv2
import depthai as dai
import time

import argparse
import signal
import threading
import numpy as np

from typing import Optional, Dict

# This example shows how to use the external frame sync node to synchronize multiple devices
# This example assumes that all devices are connected to the same host
# The script identifies the master device and slave devices and then starts the master first
# This is necessary because otherwise slave devices will not be able to detect the master FSYNC signal

SET_MANUAL_EXPOSURE = False  # Set to True to use manual exposure settings

STEREO_SOCKET = "STEREO"
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
# Create camera outputs
# ---------------------------------------------------------------------------
def createCameraOutputs(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket, sensorFps: float, role: dai.ExternalFrameSyncRole):
    cam = None

    # Only specify FPS if camera is master
    if role == dai.ExternalFrameSyncRole.MASTER:
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

    if socket == dai.CameraBoardSocket.CAM_A:
        output = (
            cam.requestOutput(
                (1920, 1080), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP
            )
        )
    else:
        output = (
            cam.requestFullResolutionOutput()
        )
    return pipeline, output

# ---------------------------------------------------------------------------
# Create synchronization node
# ---------------------------------------------------------------------------
def createSyncNode(syncThreshold: datetime.timedelta):
    global masterPipeline, masterNode, slaveQueues, inputQueues, outputNames, firstSlaveName, slavePipelines

    # If no master is specified, create a sync node on the first slave pipeline
    if firstSlaveName is None:
        sync = masterPipeline.create(dai.node.Sync)
    else:
        sync = slavePipelines[firstSlaveName].create(dai.node.Sync)

    # Sync node will run on the host, since it needs to sync multiple devices
    sync.setRunOnHost(True)
    sync.setSyncThreshold(syncThreshold)

    # Link master camera outputs to the sync node
    if not slaveOnly:
        for socketName, camOutput in masterNode.items():
            name = f"master_{socketName}"
            camOutput.link(sync.inputs[name])
            outputNames.append(name)

    # For slaves, we must create an input queue for each output
    # We will then manually forward the frames from each input queue to the output queue
    # This is because slave devices have separate pipelines from the master
    for deviceName, sockets in slaveQueues.items():
        for socketName, camOutputQueue in sockets.items():
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
    global masterNode, slaveQueues, camSockets

    pipeline, outNode = createCameraOutputs(pipeline, socket, targetFps, role)

    # Master cameras will be linked to the sync node directly
    if role == dai.ExternalFrameSyncRole.MASTER:
        if slaveOnly:
            raise RuntimeError("Slave-only specified, but master device detected")

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
    
    # Keep track of all camera socket names
    if socket.name not in camSockets:
        camSockets.append(socket.name)

    return pipeline, outNode

def setupDevice(
        stack: contextlib.ExitStack,
        deviceInfo: dai.DeviceInfo,
        targetFps: float):
    global masterPipeline, slavePipelines, slaveQueues, camSockets, slaveOnly, firstSlaveName, masterNode

    # Create pipeline for device
    pipeline = stack.enter_context(dai.Pipeline(dai.Device(deviceInfo)))
    device = pipeline.getDefaultDevice()

    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError("This example supports only RVC4 platform!")

    name = deviceInfo.getXLinkDeviceDesc().mxid
    role = device.getExternalFrameSyncRole()

    print("=== Connected to", deviceInfo.getDeviceId())
    print("    Device ID:", device.getDeviceId())
    print("    Num of cameras:", len(device.getConnectedCameras()))

    # create stereo node
    if neuralStereo:
        stereo = pipeline.create(dai.node.NeuralDepth)
    else:
        stereo = pipeline.create(dai.node.StereoDepth)

    for socket in device.getConnectedCameras():
        pipeline, outNode = setUpCameraSocket(pipeline, socket, name, targetFps, role)

        # link stereo node
        if socket == dai.CameraBoardSocket.CAM_B:
            outNode.link(stereo.left)
        elif socket == dai.CameraBoardSocket.CAM_C:
            outNode.link(stereo.right)

    if not neuralStereo:
        stereo.setRectification(True)
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)

    if role == dai.ExternalFrameSyncRole.MASTER:
        if slaveOnly:
            raise RuntimeError("Slave-only specified, but master device detected")
        
        device.setExternalStrobeEnable(True)
        print(f"{device.getDeviceId()} is master")

        if masterPipeline is not None:
            raise RuntimeError("Only one master pipeline is supported")
        
        masterPipeline = pipeline

        if masterNode is None:
            masterNode = {}

        masterNode[STEREO_SOCKET] = stereo.disparity
    elif role == dai.ExternalFrameSyncRole.SLAVE:
        slavePipelines[name] = pipeline
        print(f"{device.getDeviceId()} is slave")

        if slaveQueues.get(name) is None:
            slaveQueues[name] = {}

        slaveQueues[name][STEREO_SOCKET] = stereo.disparity.createOutputQueue()

        if firstSlaveName is None and slaveOnly:
            firstSlaveName = name
    else:
        raise RuntimeError(f"Don't know how to handle role {role}")
    
    if STEREO_SOCKET not in camSockets:
        camSockets.append(STEREO_SOCKET)

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
parser.add_argument("-d", "--devices", default=[], nargs="+", help="Device IPs or IDs", required=False)
parser.add_argument("-t1", "--recv-all-timeout-sec", type=float, default=10, help="Timeout for receiving the first frame from all devices", required=False)
parser.add_argument("-t2", "--sync-threshold-sec", type=float, default=1e-3, help="Sync threshold in seconds", required=False)
parser.add_argument("-t3", "--initial-sync-timeout-sec", type=float, default=4, help="Timeout for synchronization to complete", required=False)
parser.add_argument("-nn", "--neural-depth", action="store_true", help="Use neural depth instead", required=False)
parser.add_argument("-s", "--slave-only", action="store_true", help="Run the script without a master device", required=False)
args = parser.parse_args()

# if user did not specify device IPs, use all available devices
if len(args.devices) == 0:
    deviceInfos = dai.Device.getAllAvailableDevices()
else:
    deviceInfos = [dai.DeviceInfo(ip) for ip in args.devices]

targetFps = args.fps
recvAllTimeoutSec = args.recv_all_timeout_sec
syncThresholdSec = args.sync_threshold_sec
initialSyncTimeoutSec = args.initial_sync_timeout_sec

slaveOnly = False
if args.slave_only:
    slaveOnly = True

neuralStereo = False
if args.neural_depth:
    neuralStereo = True
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black
maxDisparity = 1

with contextlib.ExitStack() as stack:

    # Variables to keep track of master and slave pipelines and outputs
    masterPipeline: Optional[dai.Pipeline] = None
    masterNode: Optional[Dict[str, dai.Node.Output]] = None
    slavePipelines: Dict[str, dai.Pipeline] = {}
    slaveQueues: Dict[str, Dict[str, dai.MessageQueue]] = {}

    # keep track of sync node inputs for slaves
    inputQueues = {}

    # keep track of all sync node output names
    outputNames = []

    # keep track of all camera socket names
    camSockets = []

    # keep track of the first slave name if no master is specified
    firstSlaveName: Optional[str] = None

    for idx, deviceInfo in enumerate(deviceInfos):
        setupDevice(stack, deviceInfo, targetFps)

    if not slaveOnly:
        if masterPipeline is None or masterNode is None:
            raise RuntimeError("No master detected!")

    # Create sync node
    # Sync node groups the frames so that all synced frames are timestamped to within one frame time
    sync = createSyncNode(datetime.timedelta(milliseconds=1000 / (2 * targetFps)))
    queue = sync.out.createOutputQueue()

    # Start pipelines
    # The master pipeline will be started first, then the slave pipelines
    if not slaveOnly:
        masterPipeline.start()

    for k, sockets in slavePipelines.items():
        sockets.start()

    fpsCounter = FPSCounter()

    latestFrameGroup = None
    firstReceived = False
    startTime = datetime.datetime.now()
    prevReceived = datetime.datetime.now()

    initialSyncTime = None
    waitingForSync = True

    def data_collector(deviceName, socketName):
        # Send frames from slave output queues to sync node input queues
        camOutputQueue = slaveQueues[deviceName][socketName]
        while running:
            if camOutputQueue.has():
                inputQueues[f"slave_{deviceName}_{socketName}"].send(camOutputQueue.get())
            else:
                time.sleep(0.001)

    threads = {}

    for deviceName, sockets in slaveQueues.items():
        for socketName, camOutputQueue in sockets.items():
            threads[f"slave_{deviceName}_{socketName}"] = threading.Thread(target=data_collector, args=(deviceName, socketName))
            threads[f"slave_{deviceName}_{socketName}"].start()

    while running:
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
                running = False

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within syncThresholdSec.
        # -------------------------------------------------------------------
        if latestFrameGroup is not None and latestFrameGroup.getNumMessages() == len(outputNames):

            # Get timestamps for each frame
            tsValues = {}
            for name in outputNames:
                tsValues[name] = latestFrameGroup[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds()
            
            # Build individual image arrays for each camera socket
            imgs = []
            for name in camSockets:
                imgs.append([])
            fps = fpsCounter.getFps()

            # calculate the greatest time difference between all frames
            delta = max(tsValues.values()) - min(tsValues.values())

            syncStatus = abs(delta) < syncThresholdSec

            # Timeout if frames don't get synced in time
            if not syncStatus and waitingForSync:
                endTime = datetime.datetime.now()
                elapsedSec = (endTime - initialSyncTime).total_seconds()
                if elapsedSec >= initialSyncTimeoutSec:
                    print("Timeout: Didn't sync frames in time")
                    running = False

            if syncStatus and waitingForSync:
                print(f"Frame synced")
                waitingForSync = False

            # Print warning if delta is too big
            if not syncStatus and not waitingForSync:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                print("Either the signal is lost or the network is congested.")
                continue

            # Create a image frame with sync info for each output
            for outputName in outputNames:

                # Find out which camera socket this output belongs to
                idx = -1
                isStereo = False
                for i, name in enumerate(camSockets):
                    if name in outputName:
                        idx = i
                        isStereo = name == STEREO_SOCKET
                        break
                if idx == -1:
                    raise RuntimeError(f"Could not find camera socket for {outputName}")
                
                # Get frame for this output
                msg = latestFrameGroup[outputName]
                if isStereo:
                    npDisparity = msg.getFrame()
                    maxDisparity = max(maxDisparity, np.max(npDisparity))
                    frame = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
                else:
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
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                imgs[idx].append(frame)

            # Show the frame
            for i, img in enumerate(imgs):
                cv2.imshow(f"synced_view_{camSockets[i]}", cv2.hconcat(imgs[i]))

            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            running = False
            break

    for t in threads.keys():
        threads[t].join()

cv2.destroyAllWindows()
