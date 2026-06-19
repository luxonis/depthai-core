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
selectedCameraSocketOrder = None
selectedCameraSocketsByDevice = None
streamSensorResolutions = None
streamOutputSizes = None
DEFAULT_OUTPUT_SIZE = (640, 480)
STEADY_STATE_THRESHOLD_US = 500.0
STEADY_STATE_HOLD_SEC = 10.0
SETTLING_FINAL_WINDOW_SEC = 10.0
SETTLING_BAND_FRACTION = 0.02
SETTLING_BAND_REFERENCE = "peak_deviation_from_final_value"

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
    legacy_steady_state_start_sec = None
    legacy_steady_state_confirmed_at_sec = None
    legacy_time_to_steady_state_sec = None
    run_start_sec = None
    for elapsed_sec, delta_us in delta_samples:
        if delta_us < STEADY_STATE_THRESHOLD_US:
            if run_start_sec is None:
                run_start_sec = elapsed_sec
            if elapsed_sec - run_start_sec >= STEADY_STATE_HOLD_SEC:
                legacy_steady_state_start_sec = run_start_sec
                legacy_steady_state_confirmed_at_sec = elapsed_sec
                legacy_time_to_steady_state_sec = elapsed_sec
                break
        else:
            run_start_sec = None

    post_skip_samples = [(elapsed_sec, delta_us) for elapsed_sec, delta_us in delta_samples if elapsed_sec >= stats_skip_sec]
    post_skip_all_delta_us = [delta_us for _, delta_us in post_skip_samples]
    used_delta_us = [delta_us for _, delta_us in post_skip_samples if delta_us < STEADY_STATE_THRESHOLD_US]

    post_skip_all_avg_us = None
    post_skip_all_stddev_us = None
    post_skip_all_p99_us = None
    if post_skip_all_delta_us:
        post_skip_all_delta_array = np.asarray(post_skip_all_delta_us, dtype=np.float64)
        post_skip_all_avg_us = float(np.mean(post_skip_all_delta_array))
        post_skip_all_stddev_us = float(np.std(post_skip_all_delta_array))
        post_skip_all_p99_us = float(np.percentile(post_skip_all_delta_array, 99))

    avg_us = None
    stddev_us = None
    p99_us = None
    if used_delta_us:
        used_delta_array = np.asarray(used_delta_us, dtype=np.float64)
        avg_us = float(np.mean(used_delta_array))
        stddev_us = float(np.std(used_delta_array))
        p99_us = float(np.percentile(used_delta_array, 99))

    settling_final_value_us = None
    settling_band_us = None
    steady_state_start_sec = None
    steady_state_confirmed_at_sec = None
    time_to_steady_state_sec = None
    steady_state_avg_us = None
    steady_state_stddev_us = None
    steady_state_p99_us = None
    steady_state_frame_count = 0
    settling_peak_deviation_us = None
    if delta_samples:
        run_end_sec = delta_samples[-1][0]
        final_window_start_sec = max(0.0, run_end_sec - SETTLING_FINAL_WINDOW_SEC)
        final_window_delta_us = [delta_us for elapsed_sec, delta_us in delta_samples if elapsed_sec >= final_window_start_sec]
        if final_window_delta_us:
            settling_final_value_us = float(np.mean(np.asarray(final_window_delta_us, dtype=np.float64)))
            settling_peak_deviation_us = max(abs(delta_us - settling_final_value_us) for _, delta_us in delta_samples)
            settling_band_us = settling_peak_deviation_us * SETTLING_BAND_FRACTION

            for idx, (elapsed_sec, _) in enumerate(delta_samples):
                if all(abs(sample_delta_us - settling_final_value_us) <= settling_band_us for _, sample_delta_us in delta_samples[idx:]):
                    time_to_steady_state_sec = elapsed_sec
                    steady_state_start_sec = elapsed_sec
                    steady_state_confirmed_at_sec = elapsed_sec
                    break

            if time_to_steady_state_sec is not None:
                steady_state_delta_us = [delta_us for elapsed_sec, delta_us in delta_samples if elapsed_sec >= time_to_steady_state_sec]
                if steady_state_delta_us:
                    steady_state_array = np.asarray(steady_state_delta_us, dtype=np.float64)
                    steady_state_avg_us = float(np.mean(steady_state_array))
                    steady_state_stddev_us = float(np.std(steady_state_array))
                    steady_state_p99_us = float(np.percentile(steady_state_array, 99))
                    steady_state_frame_count = len(steady_state_delta_us)

    return {
        "steady_state_threshold_us": STEADY_STATE_THRESHOLD_US,
        "steady_state_hold_sec": STEADY_STATE_HOLD_SEC,
        "time_to_steady_state_sec": time_to_steady_state_sec,
        "steady_state_start_sec": steady_state_start_sec,
        "steady_state_confirmed_at_sec": steady_state_confirmed_at_sec,
        "legacy_time_to_steady_state_sec": legacy_time_to_steady_state_sec,
        "legacy_steady_state_start_sec": legacy_steady_state_start_sec,
        "legacy_steady_state_confirmed_at_sec": legacy_steady_state_confirmed_at_sec,
        "settling_final_window_sec": SETTLING_FINAL_WINDOW_SEC,
        "settling_band_fraction": SETTLING_BAND_FRACTION,
        "settling_band_reference": SETTLING_BAND_REFERENCE,
        "settling_final_value_us": settling_final_value_us,
        "settling_peak_deviation_us": settling_peak_deviation_us,
        "settling_band_us": settling_band_us,
        "steady_state_avg_us": steady_state_avg_us,
        "steady_state_stddev_us": steady_state_stddev_us,
        "steady_state_p99_us": steady_state_p99_us,
        "steady_state_frame_count": steady_state_frame_count,
        "stats_skip_sec": stats_skip_sec,
        "post_skip_total_frames": len(post_skip_samples),
        "post_skip_all_avg_us": post_skip_all_avg_us,
        "post_skip_all_stddev_us": post_skip_all_stddev_us,
        "post_skip_all_p99_us": post_skip_all_p99_us,
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
    print(f"  Sensor FPS: {summary['sensor_fps']:.3f}")
    print(
        "  Time to reach steady state: "
        f"{format_sec(summary['time_to_steady_state_sec'])} "
        f"(within {summary['settling_band_fraction'] * 100:.1f}% settling band around the final value estimate)"
    )
    print(f"  Final value estimate (last {summary['settling_final_window_sec']:.1f} s mean): {format_us(summary['settling_final_value_us'])}")
    print(f"  Settling band ({summary['settling_band_reference']}): {format_us(summary['settling_band_us'])}")
    print(f"  Avg after steady state: {format_us(summary['steady_state_avg_us'])}")
    print(f"  Std dev after steady state: {format_us(summary['steady_state_stddev_us'])}")
    print(f"  P99 after steady state: {format_us(summary['steady_state_p99_us'])}")
    print(f"  N steady-state frames: {summary['steady_state_frame_count']}")

# ---------------------------------------------------------------------------
# Create camera outputs
# ---------------------------------------------------------------------------
def buildCandidateDeviceKeys(device: dai.Device):
    deviceInfo = device.getDeviceInfo()
    candidateDeviceKeys = [device.getDeviceId()]
    if deviceInfo.deviceId:
        candidateDeviceKeys.append(deviceInfo.deviceId)
    if deviceInfo.name:
        candidateDeviceKeys.append(deviceInfo.name)
    return candidateDeviceKeys

def lookupPerStreamSize(device: dai.Device, socketName: str, mapping):
    if mapping is None:
        return None

    for deviceKey in buildCandidateDeviceKeys(device):
        value = mapping.get((deviceKey, socketName))
        if value is not None:
            return value

    return None

def createCameraOutputs(
        pipeline: dai.Pipeline,
        device: dai.Device,
        socket: dai.CameraBoardSocket,
        sensorResolution,
        outputSize,
        sensorFps: float,
        role: dai.ExternalFrameSyncRole):
    global syncType, manualExposureUs, manualIso
    cam = None

    # Only specify FPS if camera is master
    if role == dai.ExternalFrameSyncRole.MASTER or syncType == SyncType.PTP:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket, sensorResolution=sensorResolution, sensorFps=sensorFps)
        )
    # Slave cameras will lock to the master's FPS
    else:
        cam = (
            pipeline.create(dai.node.Camera)
            .build(socket, sensorResolution=sensorResolution)
        )

    output = (
        cam.requestOutput(
            outputSize, dai.ImgFrame.Type.NV12, dai.ImgResizeMode.STRETCH
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
        device: dai.Device,
        socket: dai.CameraBoardSocket,
        deviceName: str,
        targetFps: float,
        role: dai.ExternalFrameSyncRole):
    global masterNode, slaveQueues, camSockets, syncType, streamSensorResolutions, streamOutputSizes

    sensorResolution = lookupPerStreamSize(device, socket.name, streamSensorResolutions)
    outputSize = lookupPerStreamSize(device, socket.name, streamOutputSizes) or DEFAULT_OUTPUT_SIZE
    pipeline, outNode = createCameraOutputs(pipeline, device, socket, sensorResolution, outputSize, targetFps, role)

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

def getSelectedSockets(device: dai.Device):
    global selectedCameraSocketOrder, selectedCameraSocketsByDevice

    availableSockets = list(device.getConnectedCameras())
    if not availableSockets:
        raise RuntimeError("No connected cameras found on device")

    candidateDeviceKeys = buildCandidateDeviceKeys(device)

    requestedSocketOrder = None
    if selectedCameraSocketsByDevice is not None:
        for deviceKey in candidateDeviceKeys:
            if deviceKey in selectedCameraSocketsByDevice:
                requestedSocketOrder = selectedCameraSocketsByDevice[deviceKey]
                break
    elif not selectedCameraSocketOrder:
        # Preserve the current local behavior unless a socket filter is explicitly requested.
        return [availableSockets[0]]
    else:
        requestedSocketOrder = selectedCameraSocketOrder

    availableByName = {socket.name: socket for socket in availableSockets}
    missingSockets = [name for name in requestedSocketOrder if name not in availableByName]
    if missingSockets:
        availableNames = ", ".join(sorted(availableByName))
        missingNames = ", ".join(missingSockets)
        raise RuntimeError(
            f"Requested camera sockets not present on device {device.getDeviceId()}: {missingNames}. "
            f"Available sockets: {availableNames}"
        )

    return [availableByName[name] for name in requestedSocketOrder]

def parseDeviceCameraSockets(device_camera_socket_args):
    if not device_camera_socket_args:
        return None

    parsed = {}
    for item in device_camera_socket_args:
        if "=" not in item:
            raise RuntimeError(
                "Each --device-camera-sockets entry must look like DEVICE_ID_OR_IP=CAM_A,CAM_B"
            )
        deviceIdOrIp, socketList = item.split("=", 1)
        sockets = [socket.strip() for socket in socketList.split(",") if socket.strip()]
        if not deviceIdOrIp or not sockets:
            raise RuntimeError(
                "Each --device-camera-sockets entry must include a device identifier and at least one socket"
            )
        parsed[deviceIdOrIp] = sockets

    return parsed

def parsePerStreamSizeMap(arg_values, argument_name):
    if not arg_values:
        return None

    parsed = {}
    for item in arg_values:
        if "=" not in item or "/" not in item:
            raise RuntimeError(
                f"Each {argument_name} entry must look like DEVICE_ID_OR_IP/CAM_A=WIDTHxHEIGHT"
            )
        streamKey, sizeString = item.split("=", 1)
        deviceKey, socketName = streamKey.rsplit("/", 1)
        if "x" not in sizeString:
            raise RuntimeError(f"Each {argument_name} entry must use WIDTHxHEIGHT format")
        widthString, heightString = sizeString.split("x", 1)
        parsed[(deviceKey, socketName)] = (int(widthString), int(heightString))

    return parsed

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
    deviceSockets = getSelectedSockets(device)
    print("    Using cameras:", ", ".join(socket.name for socket in deviceSockets))

    for socket in deviceSockets:
        pipeline = setUpCameraSocket(pipeline, device, socket, name, targetFps, role)

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
parser.add_argument("--sensor-fps", type=float, default=None, help="Optional sensor FPS override", required=False)
parser.add_argument("-d", "--devices", default=[], nargs="+", help="Device IPs or IDs", required=False)
parser.add_argument("--camera-sockets", default=None, nargs="+", help="Optional camera sockets to compare, for example CAM_A CAM_B", required=False)
parser.add_argument("--device-camera-sockets", default=None, nargs="+", help="Optional per-device sockets, for example 192.168.10.80=CAM_A 192.168.10.76=CAM_B", required=False)
parser.add_argument("--stream-sensor-resolutions", default=None, nargs="+", help="Per-stream sensor resolutions, for example 192.168.10.80/CAM_A=8000x6000", required=False)
parser.add_argument("--stream-output-sizes", default=None, nargs="+", help="Per-stream output sizes, for example 192.168.10.80/CAM_A=8000x6000", required=False)
parser.add_argument("-t1", "--recv-all-timeout-sec", type=float, default=10, help="Timeout for receiving the first frame from all devices", required=False)
parser.add_argument("-t2", "--sync-threshold-sec", type=float, default=1e-3, help="Sync threshold in seconds", required=False)
parser.add_argument("-t3", "--initial-sync-timeout-sec", type=float, default=4, help="Timeout for synchronization to complete", required=False)
parser.add_argument("--test-duration-sec", type=float, default=120, help="Total test duration in seconds", required=False)
parser.add_argument("--stats-skip-sec", type=float, default=30, help="Ignore frames before this time for summary statistics", required=False)
parser.add_argument("--stats-file", type=str, default=None, help="Optional path to save the timestamp delta statistics JSON", required=False)
parser.add_argument("--exposure-us", type=int, default=None, help="Optional manual exposure in microseconds", required=False)
parser.add_argument("--iso", type=int, default=None, help="Optional manual ISO value", required=False)
parser.add_argument("--headless", action="store_true", help="Disable OpenCV display windows", required=False)
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

if len(deviceInfos) == 0:
    raise RuntimeError("No devices found")
selectedCameraSocketsByDevice = parseDeviceCameraSockets(args.device_camera_sockets)
if args.camera_sockets is not None and selectedCameraSocketsByDevice is not None:
    raise RuntimeError("Use either --camera-sockets or --device-camera-sockets, not both")
if len(deviceInfos) == 1 and selectedCameraSocketsByDevice is None and (args.camera_sockets is None or len(args.camera_sockets) < 2):
    raise RuntimeError("Single-device runs require --camera-sockets with at least two sockets")
streamSensorResolutions = parsePerStreamSizeMap(args.stream_sensor_resolutions, "--stream-sensor-resolutions")
streamOutputSizes = parsePerStreamSizeMap(args.stream_output_sizes, "--stream-output-sizes")

targetFps = args.fps  # Must match sensorFps in createPipeline()
sensorFps = args.sensor_fps or args.fps
recvAllTimeoutSec = args.recv_all_timeout_sec

syncThresholdSec = args.sync_threshold_sec
initialSyncTimeoutSec = args.initial_sync_timeout_sec
testDurationSec = args.test_duration_sec
statsSkipSec = args.stats_skip_sec
manualExposureUs = args.exposure_us
manualIso = args.iso
headless = args.headless
selectedCameraSocketOrder = args.camera_sockets
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
        setupDevice(stack, deviceInfo, sensorFps)

    if masterPipeline is None or masterNode is None:
        raise RuntimeError("No master detected!")

    configuredStreamCount = len(masterNode) + sum(len(sockets) for sockets in slaveQueues.values())
    if configuredStreamCount < 2:
        raise RuntimeError("Need at least two camera streams to compare")

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
                debugRows = []
                for name in outputNames:
                    frame = latestFrameGroup[name]
                    tsValues[name] = frame.getTimestamp(dai.CameraExposureOffset.END).total_seconds()
                    debugRows.append(
                        (
                            name,
                            frame.getSequenceNum(),
                            frame.getTimestamp().total_seconds(),
                            frame.getTimestamp(dai.CameraExposureOffset.END).total_seconds(),
                            frame.getTimestampDevice().total_seconds(),
                            frame.getTimestampDevice(dai.CameraExposureOffset.END).total_seconds(),
                        )
                    )

                delta = max(tsValues.values()) - min(tsValues.values())
                syncStatus = abs(delta) < syncThresholdSec
                deltaSamples.append((actualDurationSec, delta * 1e6))
                if len(deltaSamples) <= 5 or (len(deltaSamples) % 30) == 0:
                    debugSummary = " | ".join(
                        [
                            f"{name}:seq={seq},host={hostTs:.9f},hostEnd={hostEndTs:.9f},dev={devTs:.9f},devEnd={devEndTs:.9f}"
                            for name, seq, hostTs, hostEndTs, devTs, devEndTs in debugRows
                        ]
                    )
                    print(f"TSDBG example group={len(deltaSamples)} delta_us={delta * 1e6:.3f} {debugSummary}")
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
            if not headless:
                # Build individual image arrays for each camera socket, displayed side-by-side.
                imgs = [[] for _ in camSockets]

                # Create an image frame with sync info for each output.
                for outputName in outputNames:
                    # Find out which camera socket this output belongs to.
                    idx = -1
                    for i, name in enumerate(camSockets):
                        if name in outputName:
                            idx = i
                            break
                    if idx == -1:
                        raise RuntimeError(f"Could not find camera socket for {outputName}")

                    frame = latestFrameGroup[outputName].getCvFrame()

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

            latestFrameGroupDirty = False
            latestFrameGroup = None  # Wait for next batch

        if not headless and cv2.waitKey(1) & 0xFF == ord("q"):
            running = False
            break

        if actualDurationSec >= testDurationSec:
            print(f"Test duration reached: {actualDurationSec:.2f} sec")
            running = False
            break

    for t in threads.keys():
        threads[t].join()

if not headless:
    cv2.destroyAllWindows()

summary = summarize_delta_statistics(deltaSamples, statsSkipSec)
summary.update(
    {
        "run_started_at": startWallTime.isoformat(),
        "sync_type": syncType.name,
        "selected_fps": targetFps,
        "target_fps": targetFps,
        "sensor_fps": sensorFps,
        "sync_threshold_us": syncThresholdSec * 1e6,
        "test_duration_sec": testDurationSec,
        "actual_duration_sec": actualDurationSec,
        "device_ids_or_ips": args.devices,
        "camera_sockets_requested": args.camera_sockets,
        "device_camera_sockets_requested": args.device_camera_sockets,
        "stream_sensor_resolutions_requested": args.stream_sensor_resolutions,
        "stream_output_sizes_requested": args.stream_output_sizes,
        "camera_sockets_used": camSockets,
    }
)

with open(statsFilePath, "w", encoding="utf-8") as statsFile:
    json.dump(summary, statsFile, indent=2, sort_keys=True)

print_delta_statistics(summary, statsFilePath)
