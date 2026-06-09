#!/usr/bin/env python3

import contextlib
import datetime
import json

import cv2
import depthai as dai
import numpy as np
import time

import argparse
import signal
import threading

from typing import Optional, Dict
from enum import Enum

manualExposureUs = None
manualIso = None
STEADY_STATE_THRESHOLD_US = 500.0
STEADY_STATE_HOLD_SEC = 10.0

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

def summarize_delta_statistics(delta_samples, stats_skip_sec):
    steady_state_start_sec = None
    steady_state_confirmed_at_sec = None
    time_to_steady_state_sec = None
    run_start_sec = None
    for elapsed_sec, delta_us in delta_samples:
        if delta_us < STEADY_STATE_THRESHOLD_US:
            if run_start_sec is None:
                run_start_sec = elapsed_sec
            if elapsed_sec - run_start_sec >= STEADY_STATE_HOLD_SEC:
                steady_state_start_sec = run_start_sec
                steady_state_confirmed_at_sec = elapsed_sec
                time_to_steady_state_sec = elapsed_sec
                break
        else:
            run_start_sec = None

    post_skip_samples = [(elapsed_sec, delta_us) for elapsed_sec, delta_us in delta_samples if elapsed_sec >= stats_skip_sec]
    used_delta_us = [delta_us for _, delta_us in post_skip_samples if delta_us < STEADY_STATE_THRESHOLD_US]

    avg_us = None
    stddev_us = None
    p99_us = None
    if used_delta_us:
        used_delta_array = np.asarray(used_delta_us, dtype=np.float64)
        avg_us = float(np.mean(used_delta_array))
        stddev_us = float(np.std(used_delta_array))
        p99_us = float(np.percentile(used_delta_array, 99))

    return {
        "steady_state_threshold_us": STEADY_STATE_THRESHOLD_US,
        "steady_state_hold_sec": STEADY_STATE_HOLD_SEC,
        "time_to_steady_state_sec": time_to_steady_state_sec,
        "steady_state_start_sec": steady_state_start_sec,
        "steady_state_confirmed_at_sec": steady_state_confirmed_at_sec,
        "stats_skip_sec": stats_skip_sec,
        "post_skip_total_frames": len(post_skip_samples),
        "used_frame_count": len(used_delta_us),
        "p99_us": p99_us,
        "avg_us": avg_us,
        "stddev_us": stddev_us,
    }

def print_delta_statistics(summary, stats_file_path):
    def format_sec(value):
        return "not reached" if value is None else f"{value:.3f} s"

    def format_us(value):
        return "n/a" if value is None else f"{value:.3f} us"

    print("Timestamp delta statistics")
    print(f"  Stats file: {stats_file_path}")
    print(f"  Selected FPS: {summary['selected_fps']:.3f}")
    print(
        "  Time to reach steady state: "
        f"{format_sec(summary['time_to_steady_state_sec'])} "
        f"(continuous {summary['steady_state_hold_sec']:.1f} s below {summary['steady_state_threshold_us']:.0f} us)"
    )
    print(f"  Delta p99 (< {summary['steady_state_threshold_us']:.0f} us after {summary['stats_skip_sec']:.1f} s): {format_us(summary['p99_us'])}")
    print(f"  Avg (< {summary['steady_state_threshold_us']:.0f} us after {summary['stats_skip_sec']:.1f} s): {format_us(summary['avg_us'])}")
    print(f"  Std dev (< {summary['steady_state_threshold_us']:.0f} us after {summary['stats_skip_sec']:.1f} s): {format_us(summary['stddev_us'])}")
    print(f"  N frames used: {summary['used_frame_count']}")

# ---------------------------------------------------------------------------
# Create camera outputs
# ---------------------------------------------------------------------------
def createCameraOutputs(pipeline: dai.Pipeline, socket: dai.CameraBoardSocket, sensorFps: float, role: dai.ExternalFrameSyncRole):
    global syncType, manualExposureUs, manualIso
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
            (640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH
        )
    )

    if syncType == SyncType.PTP:
        cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
        print(f"Setting PTP for {socket.name}")

    if manualExposureUs is not None and manualIso is not None:
        cam.initialControl.setManualExposure(manualExposureUs, manualIso)

    return pipeline, output

# ---------------------------------------------------------------------------
# Create synchronization node
# ---------------------------------------------------------------------------
def createSyncNode(syncThreshold: datetime.timedelta):
    global masterPipeline, masterNode, masterName, slaveQueues, inputQueues, outputNames, slavePipelines
    sync = masterPipeline.create(dai.node.Sync)

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
    global masterNode, slaveQueues, camSockets, syncType
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

    for socket in device.getConnectedCameras():
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
parser.add_argument("--test-duration-sec", type=float, default=120, help="Total test duration in seconds", required=False)
parser.add_argument("--stats-skip-sec", type=float, default=30, help="Ignore frames before this time for summary statistics", required=False)
parser.add_argument("--stats-file", type=str, default=None, help="Optional path to save the timestamp delta statistics JSON", required=False)
parser.add_argument("--exposure-us", type=int, default=None, help="Optional manual exposure in microseconds", required=False)
parser.add_argument("--iso", type=int, default=None, help="Optional manual ISO value", required=False)
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--external-sync", action="store_true", help="Use external sync")
group.add_argument("--ptp-sync", action="store_true", help="Use PTP sync")
args = parser.parse_args()

if (args.exposure_us is None) != (args.iso is None):
    raise RuntimeError("Both --exposure-us and --iso must be provided together")

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
testDurationSec = args.test_duration_sec
statsSkipSec = args.stats_skip_sec
manualExposureUs = args.exposure_us
manualIso = args.iso
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
    latestFrameMetrics = None
    latestFrameGroupDirty = False
    firstReceived = False
    startWallTime = datetime.datetime.now()
    startMonotonic = time.monotonic()
    prevReceived = startMonotonic

    initialSyncTime = None
    waitingForSync = True
    deltaSamples = []
    actualDurationSec = 0.0
    statsFilePath = args.stats_file or f"multi_device_frame_sync_stats_{startWallTime.strftime('%Y%m%d_%H%M%S')}.json"

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
        nowMonotonic = time.monotonic()
        actualDurationSec = nowMonotonic - startMonotonic

        # Get frames from sync node output queue
        while queue.has():
            latestFrameGroup = queue.get()
            latestFrameGroupDirty = True
            nowMonotonic = time.monotonic()
            actualDurationSec = nowMonotonic - startMonotonic
            if not firstReceived:
                firstReceived = True
                initialSyncTime = nowMonotonic
            prevReceived = nowMonotonic
            fpsCounter.tick()
            if latestFrameGroup.getNumMessages() == len(outputNames):
                tsValues = {}
                for name in outputNames:
                    tsValues[name] = latestFrameGroup[name].getTimestampDevice(dai.CameraExposureOffset.END).total_seconds()

                delta = max(tsValues.values()) - min(tsValues.values())
                syncStatus = abs(delta) < syncThresholdSec
                deltaSamples.append((actualDurationSec, delta * 1e6))
                latestFrameMetrics = {
                    "tsValues": tsValues,
                    "delta": delta,
                    "syncStatus": syncStatus,
                    "fps": fpsCounter.getFps(),
                }
        
        # Timeout if we dont receive any frames at the beginning
        if not firstReceived:
            elapsedSec = actualDurationSec
            if elapsedSec >= recvAllTimeoutSec:
                print(f"Timeout: Didn't receive all frames in time: {elapsedSec:.2f} sec")
                running = False

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within syncThresholdSec.
        # -------------------------------------------------------------------
        if latestFrameGroupDirty and latestFrameGroup is not None and latestFrameGroup.getNumMessages() == len(outputNames) and latestFrameMetrics is not None:
            tsValues = latestFrameMetrics["tsValues"]

            # Build individual image arrays for each camera socket, displayed side-by-side
            imgs = []
            for name in camSockets:
                imgs.append([])
            fps = latestFrameMetrics["fps"]
            delta = latestFrameMetrics["delta"]
            syncStatus = latestFrameMetrics["syncStatus"]
            syncStatusStr = "in sync" if syncStatus else "out of sync"

            # Timeout if frames don't get synced in time
            if not syncStatus and waitingForSync:
                elapsedSec = nowMonotonic - initialSyncTime
                if elapsedSec >= initialSyncTimeoutSec:
                    print("Timeout: Didn't sync frames in time")
                    running = False

            if syncStatus and waitingForSync:
                print(f"Frame synced")
                waitingForSync = False

            if not syncStatus and not waitingForSync:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                latestFrameGroupDirty = False
                latestFrameGroup = None
                continue

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

            latestFrameGroupDirty = False
            latestFrameGroup = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            running = False
            break

        if actualDurationSec >= testDurationSec:
            print(f"Test duration reached: {actualDurationSec:.2f} sec")
            running = False
            break

    for t in threads.keys():
        threads[t].join()

cv2.destroyAllWindows()

summary = summarize_delta_statistics(deltaSamples, statsSkipSec)
summary.update(
    {
        "run_started_at": startWallTime.isoformat(),
        "sync_type": syncType.name,
        "selected_fps": targetFps,
        "target_fps": targetFps,
        "sync_threshold_us": syncThresholdSec * 1e6,
        "test_duration_sec": testDurationSec,
        "actual_duration_sec": actualDurationSec,
        "device_ids_or_ips": args.devices,
    }
)

with open(statsFilePath, "w", encoding="utf-8") as statsFile:
    json.dump(summary, statsFile, indent=2, sort_keys=True)

print_delta_statistics(summary, statsFilePath)
