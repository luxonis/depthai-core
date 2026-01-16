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
    DEVICE_INFOS = dai.Device.getAllAvailableDevices()
else:
    DEVICE_INFOS = [dai.DeviceInfo(ip) for ip in args.devices] #The master camera needs to be first here

assert len(DEVICE_INFOS) > 1, "At least two devices are required for this example."

TARGET_FPS = args.fps  # Must match sensorFps in createPipeline()
RECV_ALL_TIMEOUT_SEC = args.recv_all_timeout_sec

SYNC_THRESHOLD_SEC = args.sync_threshold_sec
INITIAL_SYNC_TIMEOUT_SEC = args.initial_sync_timeout_sec
# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
with contextlib.ExitStack() as stack:

    master_nodes = []
    slave_queues = []
    slave_pipelines = []
    master_pipelines = []
    device_ids = []

    input_queues = []
    slave_input_names = []
    output_names = []

    for idx, deviceInfo in enumerate(DEVICE_INFOS):
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

        pipeline, out_n = createPipeline(pipeline, socket, TARGET_FPS, role)

        if role == dai.M8FsyncRole.MASTER:
            device.setM8StrobeEnable(True)
            device.setM8StrobeLimits(0.05, 0.95)
            print(f"{device.getDeviceId()} is master")
            master_pipelines.append(pipeline)
            master_nodes.append(out_n)
        elif role == dai.M8FsyncRole.SLAVE:
            slave_pipelines.append(pipeline)
            slave_queues.append(out_n.createOutputQueue())
            print(f"{device.getDeviceId()} is slave")
        else:
            raise RuntimeError(f"Don't know how to handle role {role}")

        device_ids.append(deviceInfo.getXLinkDeviceDesc().name)

    if len(master_pipelines) > 1:
        raise RuntimeError("Multiple masters detected!")

    if len(master_pipelines) == 0:
        raise RuntimeError("No master detected!")

    if len(slave_pipelines) < 1:
        raise RuntimeError("No slaves detected!")

    sync = master_pipelines[0].create(dai.node.Sync)
    sync.setRunOnHost(True)
    sync.setSyncThreshold(datetime.timedelta(milliseconds=1000 / (2 * TARGET_FPS)))
    master_nodes[0].link(sync.inputs["master"])
    output_names.append("master")

    for i in range(len(slave_queues)):
        name = f"slave_{i}"
        slave_input_names.append(name)
        output_names.append(name)
        input_queue = sync.inputs[name].createInputQueue()
        input_queues.append(input_queue)

    queue = sync.out.createOutputQueue()

    for p in master_pipelines:
        p.start()

    for p in slave_pipelines:
        p.start()


    fpsCounter = FPSCounter()

    latest_frame_group = None
    first_received = False
    start_time = datetime.datetime.now()
    prev_received = datetime.datetime.now()

    initial_sync_time = None
    waiting_for_sync = True

    while running:

        for i, slq in enumerate(slave_queues):
            while slq.has():
                input_queues[i].send(slq.get())

        while queue.has():
            latest_frame_group = queue.get()
            if not first_received:
                first_received = True
                initial_sync_time = datetime.datetime.now()
            prev_received = datetime.datetime.now()
            fpsCounter.tick()

        if not first_received:
            end_time = datetime.datetime.now()
            elapsed_sec = (end_time - start_time).total_seconds()
            if elapsed_sec >= RECV_ALL_TIMEOUT_SEC:
                print(f"Timeout: Didn't receive all frames in time: {elapsed_sec:.2f} sec")
                running = False

        # -------------------------------------------------------------------
        # Synchronise: we need at least one frame from every camera and their
        # timestamps must align within SYNC_THRESHOLD_SEC.
        # -------------------------------------------------------------------
        if latest_frame_group is not None and latest_frame_group.getNumMessages() == len(output_names):
            ts_values = [latest_frame_group[name].getTimestamp(dai.CameraExposureOffset.END).total_seconds() for name in output_names]
            # Build composite image side‑by‑side
            imgs = []
            fps = fpsCounter.getFps()

            for i, output_name in enumerate(output_names):
                msg = latest_frame_group[output_name]
                frame = msg.getCvFrame()
                cv2.putText(
                    frame,
                    f"{device_ids[i]} ({output_name})",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 127, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"Timestamp: {ts_values[i]} | FPS:{fps:.2f}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 0, 50),
                    2,
                    cv2.LINE_AA,
                )
                imgs.append(frame)

            delta = max(ts_values) - min(ts_values)

            sync_status = abs(delta) < SYNC_THRESHOLD_SEC
            sync_status_str = "in sync" if sync_status else "out of sync"

            if not sync_status and waiting_for_sync:
                end_time = datetime.datetime.now()
                elapsed_sec = (end_time - initial_sync_time).total_seconds()
                if elapsed_sec >= INITIAL_SYNC_TIMEOUT_SEC:
                    print("Timeout: Didn't sync frames in time")
                    running = False

            if sync_status and waiting_for_sync:
                print(f"Sync status: {sync_status_str}")
                waiting_for_sync = False

            if not sync_status and not waiting_for_sync:
                print(f"Sync error: Sync lost, threshold exceeded {delta * 1e6} us")
                running = False

            color = (0, 255, 0) if sync_status_str == "in sync" else (0, 0, 255)
            
            cv2.putText(
                imgs[0],
                f"{sync_status_str} | delta = {delta*1e3:.3f} ms",
                (20, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

            cv2.imshow("synced_view", cv2.hconcat(imgs))

            latest_frame_group = None  # Wait for next batch

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
