#!/usr/bin/env python3
"""Multi-device PointCloud example: one merged cloud from the depth of several devices.

Every device computes depth aligned to its color camera. Each depth + color pair is linked to the
same PointCloud node through getDepthInput(<device id>) / getColorInput(<device id>); the node
synchronizes the streams and merges them into a single PointCloudData expressed in the common
origin of a multi-device calibration (create one with MultiDevice/multi_device_calibration.py).

The merged cloud is shown in the DepthAI visualizer: open http://localhost:8082 and press Q to quit.
"""

import argparse
from datetime import timedelta
from pathlib import Path

import depthai as dai

FPS = 10

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("-d", "--devices", nargs="+", default=[], help="Device IDs or IPs (default: the first two available devices)")
parser.add_argument(
    "-c", "--calibration", type=Path, default=Path(__file__).with_name("multi_device_calibration.json"), help="Multi-device calibration JSON"
)
parser.add_argument(
    "--size", type=int, nargs=2, default=(640, 400), metavar=("W", "H"), help="Depth / color size per device; lower it when many devices share one network link (default: 640 400)"
)
args = parser.parse_args()
SIZE = tuple(args.size)

deviceInfos = [dai.DeviceInfo(d) for d in args.devices] or dai.Device.getAllAvailableDevices()[:2]
if len(deviceInfos) < 2:
    raise SystemExit("At least two devices are required for this example.")
if not args.calibration.is_file():
    raise SystemExit(f"Multi-device calibration not found: {args.calibration}")

calibration = dai.beta.MultiDeviceCalibrationHandler(args.calibration)

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    pipeline.setMultiDeviceCalibration(calibration.getGraph())

    # One PointCloud node merges every device's depth stream
    pc = pipeline.create(dai.node.PointCloud)
    pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
    pc.sync.setSyncThreshold(timedelta(milliseconds=1000 / FPS))  # one frame period: the devices are not hardware-synchronized

    for info in deviceInfos:
        device = pipeline.addDevice(info)
        deviceId = device.getDeviceId()

        colorSockets = device.getConnectedCameras(dai.CameraSensorType.COLOR)
        colorSocket = colorSockets[0] if colorSockets else dai.CameraBoardSocket.CAM_A
        color = pipeline.create(dai.node.Camera, device).build(colorSocket, sensorFps=FPS)
        colorOut = color.requestOutput(SIZE, type=dai.ImgFrame.Type.RGB888i, fps=FPS, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

        depth = pipeline.create(dai.node.Depth, device).build(dai.node.Depth.Algorithm.AUTO, FPS, SIZE)
        depth.setAlignTo(colorOut)

        depth.depth.link(pc.getDepthInput(deviceId))
        colorOut.link(pc.getColorInput(deviceId))
        print(f"Device {deviceId}: linked as point cloud stream '{deviceId}'")

    q = pc.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)
    # Non-blocking visualizer queue: a slow browser must never stall the PointCloud node
    remote = dai.RemoteConnection()
    visualizerQueue = remote.addTopic("merged_point_cloud", "3d", maxSize=2, blocking=False)

    pipeline.start()
    remote.registerPipeline(pipeline)
    print("Merged point cloud streams:", pc.getDepthInputNames())

    try:
        while pipeline.isRunning():
            pcd = q.get()
            visualizerQueue.send(pcd)
            print(
                f"Points: {pcd.getWidth() * pcd.getHeight()}, color={pcd.isColor()}, "
                f"X=[{pcd.getMinX():.2f}, {pcd.getMaxX():.2f}] Y=[{pcd.getMinY():.2f}, {pcd.getMaxY():.2f}] Z=[{pcd.getMinZ():.2f}, {pcd.getMaxZ():.2f}] m"
            )
            if remote.waitKey(1) == ord("q"):
                break
    except KeyboardInterrupt:
        pass
