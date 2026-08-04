#!/usr/bin/env python3
"""Load a physical CAM_B/C rig, detect the floor plane, and show live stitching."""
import argparse
import time
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


DEFAULT_DEVICES = ("10.11.0.128", "10.11.0.48", "10.11.103.135")
STEREO_SOCKETS = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
ALLOWED_SOCKET_NAMES = ("CAM_B", "CAM_C")


@dataclass
class PlaneEstimate:
    point_cm: np.ndarray
    normal: np.ndarray
    inliers: int
    candidates: int
    inlier_mask: np.ndarray | None


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", action="append", help="Device IP or MX ID. Repeat in physical rig order.")
    parser.add_argument("-o", "--output-dir", type=Path, default=Path(".codex-tmp/multidevice_live_stitch"))
    parser.add_argument("--rig", type=Path, required=True, help="Rig json from multi_device_physical_calibration.py.")
    parser.add_argument("--stitched-output", type=Path, help="Saved stitched image. Defaults to <output-dir>/floor_planar_stitch.png")
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 800), help="Camera output resolution.")
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--warmup-frames", type=int, default=15, help="Drop this many startup frames before floor/stitch use.")
    parser.add_argument("--headless", action="store_true", help="Save one stitched frame and exit.")
    parser.add_argument("--stitch-socket", action="append", choices=ALLOWED_SOCKET_NAMES, help="Repeat to choose stitch sockets. Defaults to CAM_B and CAM_C.")
    parser.add_argument("--range", type=float, default=600.0, help="Maximum rendered range from cameras, in cm.")
    parser.add_argument("--view-size", type=int, nargs=2, default=(1600, 1600), help="Maximum auto-computed output view size.")
    parser.add_argument("--crop-output", action="store_true", help="Crop saved stitched frame to the non-black region.")
    parser.add_argument("--normalize-output", action="store_true", help="Contrast-stretch saved stitched frame.")

    parser.add_argument("--floor-device-index", type=int, default=0, help="Device index used for CAM_B/C stereo floor detection.")
    parser.add_argument("--settle", type=float, default=1.0, help="Minimum warm-up seconds after each camera pipeline starts.")
    parser.add_argument("--plane-frames", type=int, default=3, help="Point-cloud frames to drain before fitting the latest one.")
    parser.add_argument("--floor-roi-start", type=float, default=0.35, help="Use image rows below this fraction for floor candidates.")
    parser.add_argument("--min-z", type=float, default=30.0, help="Nearest accepted point depth, in cm.")
    parser.add_argument("--max-z", type=float, default=600.0, help="Farthest accepted point depth, in cm.")
    parser.add_argument("--min-floor-y", type=float, default=0.0, help="Minimum camera-frame Y for floor candidates, in cm.")
    parser.add_argument("--plane-threshold", type=float, default=3.0, help="RANSAC inlier threshold, in cm.")
    parser.add_argument("--ransac-iterations", type=int, default=900)
    parser.add_argument("--max-plane-points", type=int, default=30000)
    parser.add_argument("--min-normal-y", type=float, default=0.25, help="Reject planes whose normal is too vertical in CAM_B frame.")
    parser.add_argument("--min-inliers", type=int, default=500)
    return parser.parse_args()


def socket_name(socket):
    return getattr(socket, "name", str(socket).split(".")[-1])


def require_cam_bc(devices):
    required = set(STEREO_SOCKETS)
    for device in devices:
        missing = required - set(device.getConnectedCameras())
        if missing:
            missing_names = ", ".join(socket_name(socket) for socket in sorted(missing, key=socket_name))
            raise SystemExit(f"Device {device.getDeviceId()} is missing {missing_names}")


def warmup_seconds(args):
    if args.fps <= 0.0:
        return args.settle
    return max(args.settle, max(0, args.warmup_frames) / args.fps)


def open_gates_after_warmup(gate_queues, args, label):
    if not gate_queues:
        return
    seconds = warmup_seconds(args)
    print(f"Warming up {label}: dropping startup frames for {seconds:.1f}s.", flush=True)
    if seconds > 0.0:
        time.sleep(seconds)
    for queue in gate_queues:
        queue.send(dai.GateControl.openGate())


def build_camera_outputs(pipeline, devices, sockets, resolution, fps, gated=False):
    outputs = []
    gates = []
    for device in devices:
        for socket in sockets:
            camera = pipeline.create(dai.node.Camera, device).build(socket)
            output = camera.requestOutput(tuple(resolution), fps=fps)
            if gated:
                gate = pipeline.create(dai.node.Gate, device)
                gate.initialConfig = dai.GateControl.closeGate()
                output.link(gate.input)
                output = gate.output
                gates.append(gate)
            outputs.append(output)
    return outputs, gates


def colorize_depth(depth):
    if depth is None:
        return None
    sampled = depth[::4, ::4]
    valid = sampled[sampled > 0]
    if valid.size == 0:
        scaled = np.zeros_like(depth, dtype=np.uint8)
    else:
        low, high = np.percentile(valid, (2.0, 98.0))
        if high <= low:
            high = low + 1.0
        scaled = np.clip((depth.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
    return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)


def save_floor_debug(output_dir, left, right, depth, estimate):
    output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_dir / "floor_reference_CAM_B.png"), left)
    cv2.imwrite(str(output_dir / "floor_reference_CAM_C.png"), right)

    depth_color = colorize_depth(depth)
    if depth_color is not None:
        cv2.imwrite(str(output_dir / "floor_depth.png"), depth_color)
        if estimate.inlier_mask is not None and estimate.inlier_mask.shape[:2] == depth_color.shape[:2]:
            overlay = depth_color.copy()
            overlay[estimate.inlier_mask] = (0, 255, 0)
            cv2.addWeighted(overlay, 0.45, depth_color, 0.55, 0.0, overlay)
            cv2.imwrite(str(output_dir / "floor_plane_inliers.png"), overlay)


def fit_plane_ransac(points, args):
    rng = np.random.default_rng(7)
    if len(points) > args.max_plane_points:
        sampled = points[rng.choice(len(points), args.max_plane_points, replace=False)]
    else:
        sampled = points

    best_normal = None
    best_point = None
    best_count = 0
    for _ in range(args.ransac_iterations):
        tri = sampled[rng.choice(len(sampled), 3, replace=False)]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal = normal / norm
        if abs(normal[1]) < args.min_normal_y:
            continue
        distances = np.abs((sampled - tri[0]) @ normal)
        count = int(np.count_nonzero(distances < args.plane_threshold))
        if count > best_count:
            best_normal = normal
            best_point = tri[0]
            best_count = count

    if best_normal is None:
        raise RuntimeError("Could not find a floor-like plane in the reference CAM_B/C point cloud.")

    inliers = sampled[np.abs((sampled - best_point) @ best_normal) < args.plane_threshold]
    if len(inliers) < 3:
        raise RuntimeError("The best floor plane had too few inliers to refine.")

    center = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal[1] > 0.0:
        normal = -normal

    distances = np.abs((points - center) @ normal)
    all_inliers = distances < args.plane_threshold
    if int(np.count_nonzero(all_inliers)) < args.min_inliers:
        raise RuntimeError(f"Only {int(np.count_nonzero(all_inliers))} floor inliers found; need at least {args.min_inliers}.")

    refined = points[all_inliers]
    center = refined.mean(axis=0)
    _, _, vh = np.linalg.svd(refined - center, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal[1] > 0.0:
        normal = -normal

    all_inliers = np.abs((points - center) @ normal) < args.plane_threshold
    return center, normal, all_inliers


def estimate_floor_plane(point_cloud, args):
    points = np.asarray(point_cloud.getPoints(), dtype=np.float32)
    width = int(point_cloud.getWidth())
    height = int(point_cloud.getHeight())
    organized = width > 0 and height > 1 and width * height == len(points)

    inlier_image_mask = None
    if organized:
        points_organized = points.reshape((height, width, 3))
        rows = np.arange(height, dtype=np.int32)[:, None]
        candidate_mask = rows >= int(height * args.floor_roi_start)
        candidate_mask = candidate_mask & np.isfinite(points_organized).all(axis=2)
        candidate_mask = candidate_mask & (points_organized[:, :, 2] > args.min_z)
        candidate_mask = candidate_mask & (points_organized[:, :, 2] < args.max_z)
        candidate_mask = candidate_mask & (points_organized[:, :, 1] > args.min_floor_y)
        candidates = points_organized[candidate_mask]
    else:
        candidate_mask = None
        finite = np.isfinite(points).all(axis=1)
        finite = finite & (points[:, 2] > args.min_z)
        finite = finite & (points[:, 2] < args.max_z)
        finite = finite & (points[:, 1] > args.min_floor_y)
        candidates = points[finite]

    if len(candidates) < args.min_inliers:
        raise RuntimeError(f"Only {len(candidates)} candidate floor points found; need at least {args.min_inliers}.")

    center, normal, inlier_flags = fit_plane_ransac(candidates, args)
    if organized and candidate_mask is not None:
        inlier_image_mask = np.zeros(candidate_mask.shape, dtype=bool)
        candidate_indices = np.flatnonzero(candidate_mask.reshape(-1))
        inlier_image_mask.reshape(-1)[candidate_indices[inlier_flags]] = True

    return PlaneEstimate(
        point_cm=center.astype(np.float32),
        normal=normal.astype(np.float32),
        inliers=int(np.count_nonzero(inlier_flags)),
        candidates=len(candidates),
        inlier_mask=inlier_image_mask,
    )


def capture_floor_point_cloud(device_name, args, output_dir):
    with dai.Device(dai.DeviceInfo(device_name)) as device:
        require_cam_bc([device])
        print(f"Detecting floor from {device_name} -> {device.getDeviceId()} CAM_B/C.", flush=True)
        with dai.Pipeline(device) as pipeline:
            left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            stereo = pipeline.create(dai.node.StereoDepth)
            point_cloud = pipeline.create(dai.node.PointCloud)

            left_out = left.requestOutput(tuple(args.resolution), fps=args.fps)
            right_out = right.requestOutput(tuple(args.resolution), fps=args.fps)
            left_out.link(stereo.left)
            right_out.link(stereo.right)

            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
            stereo.setRectification(True)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)

            point_cloud.setRunOnHost(True)
            point_cloud.useCPUMT(4)
            point_cloud.initialConfig.setOrganized(True)
            point_cloud.initialConfig.setLengthUnit(dai.LengthUnit.CENTIMETER)
            point_cloud.initialConfig.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_B)
            stereo.depth.link(point_cloud.inputDepth)

            q_left = left_out.createOutputQueue(maxSize=4, blocking=False)
            q_right = right_out.createOutputQueue(maxSize=4, blocking=False)
            q_depth = point_cloud.passthroughDepth.createOutputQueue(maxSize=4, blocking=False)
            q_cloud = point_cloud.outputPointCloud.createOutputQueue(maxSize=4, blocking=False)

            pipeline.start()
            seconds = warmup_seconds(args)
            print(f"Warming up floor detector: dropping startup frames for {seconds:.1f}s.", flush=True)
            if seconds > 0.0:
                time.sleep(seconds)
            for queue in (q_left, q_right, q_depth, q_cloud):
                queue.tryGetAll()

            left_frame = right_frame = depth_frame = cloud = None
            for _ in range(max(1, args.plane_frames)):
                left_msg = q_left.get()
                right_msg = q_right.get()
                depth_msg = q_depth.get()
                cloud = q_cloud.get()
                left_frame = left_msg.getCvFrame()
                right_frame = right_msg.getCvFrame()
                depth_frame = depth_msg.getCvFrame()

    estimate = estimate_floor_plane(cloud, args)
    print(
        "Floor plane in reference CAM_B frame (cm): "
        f"point=({estimate.point_cm[0]:.1f}, {estimate.point_cm[1]:.1f}, {estimate.point_cm[2]:.1f}) "
        f"normal=({estimate.normal[0]:.4f}, {estimate.normal[1]:.4f}, {estimate.normal[2]:.4f}) "
        f"inliers={estimate.inliers}/{estimate.candidates}",
        flush=True,
    )
    save_floor_debug(output_dir, left_frame, right_frame, depth_frame, estimate)
    return estimate


def prepare_output_frame(image, crop_output, normalize_output):
    if not crop_output and not normalize_output:
        return image

    output = image.copy()
    if crop_output:
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) if output.ndim == 3 else output
        mask = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)[1]
        points = cv2.findNonZero(mask)
        if points is not None:
            x, y, width, height = cv2.boundingRect(points)
            padding = 16
            x0 = max(0, x - padding)
            y0 = max(0, y - padding)
            x1 = min(output.shape[1], x + width + padding)
            y1 = min(output.shape[0], y + height + padding)
            output = output[y0:y1, x0:x1]

    if normalize_output:
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY) if output.ndim == 3 else output
        mask = gray > 1
        samples = output[mask]
        if samples.size:
            low, high = np.percentile(samples, (2.0, 99.0))
            if high > low:
                output = np.clip((output.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)

    return output


def run_stitching(device_names, args, rig_path, plane):
    stitch_socket_names = args.stitch_socket or ["CAM_B", "CAM_C"]
    stitch_sockets = [dai.CameraBoardSocket.__members__[name] for name in stitch_socket_names]
    output_path = args.stitched_output or (args.output_dir / "floor_planar_stitch.png")

    with ExitStack() as stack:
        devices = [stack.enter_context(dai.Device(dai.DeviceInfo(name))) for name in device_names]
        require_cam_bc(devices)

        with dai.Pipeline(createImplicitDevice=False) as pipeline:
            pipeline.setMultiDeviceCalibration(dai.MultiDeviceCalibrationHandler(str(rig_path)))
            reference = dai.CoordinateFrame(devices[0].getDeviceId(), dai.CameraBoardSocket.CAM_B)
            outputs, gates = build_camera_outputs(pipeline, devices, stitch_sockets, args.resolution, args.fps, gated=True)

            unified = pipeline.create(dai.node.CoordinateFrameTransform).build(outputs, reference)
            stitching_inputs = [unified.outputs[f"output{index}"] for index in range(len(outputs))]
            stitching = pipeline.create(dai.node.Stitching).build(stitching_inputs)
            stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
            stitching.setPlane(
                dai.Point3f(float(plane.point_cm[0]), float(plane.point_cm[1]), float(plane.point_cm[2])),
                dai.Point3f(float(plane.normal[0]), float(plane.normal[1]), float(plane.normal[2])),
                dai.LengthUnit.CENTIMETER,
            )
            stitching.setMaxRange(args.range, dai.LengthUnit.CENTIMETER)
            stitching.setMaxViewSize(*args.view_size)
            stitching.setSyncThreshold(timedelta(seconds=max(0.5, 3.0 / args.fps)))

            stitched_queue = stitching.out.createOutputQueue()
            gate_queues = [gate.inputControl.createInputQueue() for gate in gates]
            pipeline.start()
            open_gates_after_warmup(gate_queues, args, "stitching")
            print(f"Planar stitching {len(outputs)} CAM_B/C streams on the detected floor plane.", flush=True)

            while pipeline.isRunning():
                stitched = stitched_queue.get().getCvFrame()
                if args.headless:
                    saved = prepare_output_frame(stitched, args.crop_output, args.normalize_output)
                    output_path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(output_path), saved)
                    print(f"Saved stitched frame {saved.shape[1]}x{saved.shape[0]} to {output_path}", flush=True)
                    return True

                cv2.imshow("CAM_B/C floor planar stitch", stitched)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("s"):
                    saved = prepare_output_frame(stitched, args.crop_output, args.normalize_output)
                    output_path.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(output_path), saved)
                    print(f"Saved stitched frame {saved.shape[1]}x{saved.shape[0]} to {output_path}", flush=True)
                if key == ord("q"):
                    return True

    return False


def main():
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    device_names = args.device or list(DEFAULT_DEVICES)

    if not args.rig.exists():
        raise SystemExit(f"Rig file does not exist: {args.rig}")
    if args.floor_device_index < 0 or args.floor_device_index >= len(device_names):
        raise SystemExit(f"--floor-device-index must be in [0, {len(device_names) - 1}]")

    plane = capture_floor_point_cloud(device_names[args.floor_device_index], args, args.output_dir)
    if not run_stitching(device_names, args, args.rig, plane):
        raise SystemExit(1)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
