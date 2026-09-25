#!/usr/bin/env python3
"""One merged point cloud from the depth of several devices in ONE dai.Pipeline.

Every device computes depth aligned to its color camera. Each depth (+ color) pair is linked to the
same PointCloud node through getDepthInput(<device id>) / getColorInput(<device id>). The node
synchronizes the streams, deprojects each with its own intrinsics, transforms it with its frame
extrinsics and merges everything into a single PointCloudData.

The frame extrinsics of a device point at its own calibration origin, so a multi-device calibration
(see multi_device_calibration.py) is loaded into the pipeline: the devices rebase their frame
extrinsics onto the common origin and the merged cloud is expressed in that coordinate system.

The merged cloud is published to the DepthAI visualizer (open http://localhost:8082) and, with
--top-view, drawn as a top-down map in an OpenCV window.
"""

import argparse
import os
import time
from datetime import timedelta
from pathlib import Path

import depthai as dai
import numpy as np

DEFAULT_CALIBRATION = Path(__file__).with_name("multi_device_calibration.json")
VISUALIZER_GROUP = "3d"
VISUALIZER_TOPIC = "merged_point_cloud"
REPORT_INTERVAL_S = 2.0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-d", "--devices", nargs="+", default=[], help="Device IDs or IPs. Defaults to the first two available devices.")
    parser.add_argument("-c", "--calibration", type=Path, default=DEFAULT_CALIBRATION, help=f"Multi-device calibration JSON (default: {DEFAULT_CALIBRATION})")
    parser.add_argument("--fps", type=float, default=10.0, help="Camera FPS (default: 10)")
    parser.add_argument("--size", type=int, nargs=2, default=(640, 400), metavar=("W", "H"), help="Depth / color size (default: 640 400)")
    parser.add_argument(
        "--algorithm", choices=["auto", "stereo", "neural"], default="auto", help="Depth algorithm for every device (default: auto)"
    )
    parser.add_argument("--no-color", action="store_true", help="Depth-only point cloud (no RGB camera stream)")
    parser.add_argument(
        "--sync-threshold-ms",
        type=float,
        default=None,
        help="Maximum timestamp spread between the depth frames of one merged cloud (default: half a frame period)",
    )
    parser.add_argument("--ptp", action="store_true", help="Hardware-sync the cameras of all devices with PTP (TIME_PTP frame sync mode)")
    parser.add_argument("--http-port", type=int, default=8082, help="DepthAI visualizer HTTP port (default: 8082)")
    parser.add_argument("--websocket-port", type=int, default=8765, help="DepthAI visualizer WebSocket port (default: 8765)")
    parser.add_argument("--no-visualizer", action="store_true", help="Do not start the DepthAI visualizer")
    parser.add_argument("--top-view", action="store_true", help="Also draw a top-down map of the merged cloud in an OpenCV window")
    parser.add_argument("--seconds", type=float, default=0.0, help="Run duration in seconds, 0 = until stopped (default: 0)")
    return parser.parse_args()


def depth_algorithm(name):
    return {
        "auto": dai.node.Depth.Algorithm.AUTO,
        "stereo": dai.node.Depth.Algorithm.STEREO,
        "neural": dai.node.Depth.Algorithm.NEURAL,
    }[name]


def draw_top_view(point_cloud, size_px=600, range_m=6.0):
    """Top-down (x/z) map of the merged cloud; the common origin is at the bottom center."""
    import cv2

    if point_cloud.isColor():
        points, colors = point_cloud.getPointsRGB()
        colors = colors[:, :3][:, ::-1]  # RGB -> BGR
    else:
        points = point_cloud.getPoints()
        colors = None

    image = np.zeros((size_px, size_px, 3), dtype=np.uint8)
    if len(points) == 0:
        return image
    scale = size_px / range_m
    px = np.rint(points[:, 0] * scale + size_px / 2).astype(np.int64)
    py = np.rint(size_px - 1 - points[:, 2] * scale).astype(np.int64)
    inside = (px >= 0) & (px < size_px) & (py >= 0) & (py < size_px) & (points[:, 2] > 0)
    if colors is None:
        image[py[inside], px[inside]] = (0, 200, 255)
    else:
        image[py[inside], px[inside]] = colors[inside]
    cv2.circle(image, (size_px // 2, size_px - 1), 6, (0, 0, 255), -1)
    cv2.putText(image, f"{range_m:.0f} m x {range_m:.0f} m, {len(points)} points", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def main():
    args = parse_args()
    if not args.calibration.is_file():
        raise FileNotFoundError(f"Multi-device calibration not found: {args.calibration} (create it with multi_device_calibration.py)")

    if args.devices:
        device_infos = [dai.DeviceInfo(d) for d in args.devices]
    else:
        device_infos = dai.Device.getAllAvailableDevices()[:2]
    if len(device_infos) < 2:
        print("At least two devices are required for this example.")
        raise SystemExit(0)

    handler = dai.beta.MultiDeviceCalibrationHandler(args.calibration)
    size = tuple(args.size)
    sync_threshold_ms = args.sync_threshold_ms if args.sync_threshold_ms is not None else 500.0 / args.fps

    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        pipeline.setMultiDeviceCalibration(handler.getGraph())

        depth_nodes = []
        devices = []
        for info in device_infos:
            device = pipeline.addDevice(info)
            device_id = device.getDeviceId()
            if handler.getDeviceSocket(device_id) is None:
                raise RuntimeError(f"Device {device_id} is not part of the calibration {args.calibration}")
            devices.append(device)

            depth = pipeline.create(dai.node.Depth, device).build(depth_algorithm(args.algorithm), args.fps, size)
            color_out = None
            if not args.no_color:
                color_sockets = device.getConnectedCameras(dai.CameraSensorType.COLOR)
                color_socket = color_sockets[0] if color_sockets else dai.CameraBoardSocket.CAM_A
                color = pipeline.create(dai.node.Camera, device).build(color_socket, sensorFps=args.fps)
                if args.ptp:
                    color.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.TIME_PTP)
                color_out = color.requestOutput(
                    size, type=dai.ImgFrame.Type.RGB888i, fps=args.fps, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True
                )
                depth.setAlignTo(color_out)
            depth_nodes.append((device_id, depth, color_out))
            print(f"Device {device_id}: depth {size[0]}x{size[1]} @ {args.fps:g} FPS" + ("" if args.no_color else f", color from {color_socket.name}"))

        # One PointCloud node on the host merges every device's depth stream
        pc = pipeline.create(dai.node.PointCloud)
        pc.initialConfig.setLengthUnit(dai.LengthUnit.METER)
        pc.sync.setSyncThreshold(timedelta(milliseconds=sync_threshold_ms))
        for device_id, depth, color_out in depth_nodes:
            depth.depth.link(pc.getDepthInput(device_id))
            if color_out is not None:
                color_out.link(pc.getColorInput(device_id))
        print(f"Depth streams: {pc.getDepthInputNames()} (sync threshold {sync_threshold_ms:g} ms)")

        queue = pc.outputPointCloud.createOutputQueue(maxSize=2, blocking=False)

        remote = None
        visualizer_queue = None
        if not args.no_visualizer:
            remote = dai.RemoteConnection(webSocketPort=args.websocket_port, httpPort=args.http_port)
            visualizer_queue = remote.addTopic(VISUALIZER_TOPIC, VISUALIZER_GROUP, maxSize=2, blocking=False)

        pipeline.start()
        if remote is not None:
            remote.registerPipeline(pipeline)
            print(f"DepthAI visualizer: http://localhost:{args.http_port}  (topic '{VISUALIZER_TOPIC}')")
        if args.top_view:
            import cv2
        print("Press Q (visualizer / OpenCV window) or Ctrl+C to stop.")

        origin = None
        frames = 0
        report_frames = 0
        report_started = time.monotonic()
        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        try:
            while pipeline.isRunning() and (deadline is None or time.monotonic() < deadline):
                point_cloud = queue.tryGet()
                if point_cloud is not None:
                    frames += 1
                    report_frames += 1
                    if origin is None:
                        extrinsics = point_cloud.getTransformation().getExtrinsics()
                        origin = f"{extrinsics.toDeviceId}/{extrinsics.toCameraSocket.name}"
                        print(f"Merged cloud expressed in the coordinate system of {origin}")
                    if visualizer_queue is not None:
                        visualizer_queue.send(point_cloud)
                    if args.top_view:
                        cv2.imshow("Merged point cloud (top view)", draw_top_view(point_cloud))

                now = time.monotonic()
                if now - report_started >= REPORT_INTERVAL_S and point_cloud is not None:
                    n = point_cloud.getWidth() * point_cloud.getHeight()
                    print(
                        f"Merged cloud: {report_frames / (now - report_started):.1f} FPS, {n} points, color={point_cloud.isColor()}, "
                        f"X [{point_cloud.getMinX():.2f}, {point_cloud.getMaxX():.2f}] "
                        f"Y [{point_cloud.getMinY():.2f}, {point_cloud.getMaxY():.2f}] "
                        f"Z [{point_cloud.getMinZ():.2f}, {point_cloud.getMaxZ():.2f}] m"
                    )
                    report_started = now
                    report_frames = 0

                key = -1
                if remote is not None:
                    key = remote.waitKey(1)
                if args.top_view:
                    key = max(key, cv2.waitKey(1))
                if key == ord("q"):
                    break
                if point_cloud is None and remote is None and not args.top_view:
                    time.sleep(0.005)
        except KeyboardInterrupt:
            pass
        finally:
            pipeline.stop()
            if remote is not None:
                remote.removeTopic(VISUALIZER_TOPIC)
            if args.top_view:
                cv2.destroyAllWindows()
        print(f"Merged point-cloud frames: {frames}")


if __name__ == "__main__":
    main()
