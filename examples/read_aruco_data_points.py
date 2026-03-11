#!/usr/bin/env python3

import argparse
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import depthai as dai
import numpy as np

try:
    from mcap.reader import make_reader
except ModuleNotFoundError as exc:
    raise RuntimeError("mcap package is required. Install it in the same environment used for this script.") from exc


CAMERA_PATTERN = re.compile(r"(^|_)(CAM_[A-Z](?:_R)?)(?=_|$)")
CAMERA_ORDER = ["CAM_A", "CAM_B", "CAM_C", "CAM_D", "CAM_E", "CAM_B_R", "CAM_C_R"]
POINT_RADIUS = 2
DEPTH_LABEL_POINT_STRIDE = 4
VIDEO_EXTENSIONS = {".mp4", ".avi"}


@dataclass(frozen=True)
class CameraReplayInput:
    stream_id: str
    camera: str
    video: Path
    metadata: Path | None


@dataclass(frozen=True)
class PointsFrame:
    reference_camera: str
    sequence_num: int
    timestamp_device_ns: int
    points: list[dict[str, Any]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read capture folder produced by new_save_aruco.py, replay all camera videos, "
            "and visualize reprojections from the reference camera to every other camera."
        )
    )
    parser.add_argument(
        "capture_folder",
        nargs="?",
        default="capture/default",
        help="Folder with *_CAM_*.mp4/.avi, *_metadata.mcap and *ARUCO_POINTS.mcap files.",
    )
    parser.add_argument(
        "--points-mcap",
        type=Path,
        default=None,
        help="Optional explicit path to reference_ARUCO_POINTS.mcap.",
    )
    parser.add_argument(
        "--target-points-mcap",
        type=Path,
        default=None,
        help="Optional explicit path to target ARUCO points MCAP.",
    )
    parser.add_argument(
        "--depth-mcap",
        type=Path,
        default=None,
        help="Optional explicit path to depth MCAP (for depth visualization).",
    )
    return parser.parse_args()


def camera_sort_key(camera: str) -> tuple[int, str]:
    if camera in CAMERA_ORDER:
        return (CAMERA_ORDER.index(camera), camera)
    return (len(CAMERA_ORDER), camera)


def detect_camera_name(path: Path) -> str | None:
    match = CAMERA_PATTERN.search(path.stem)
    if match is None:
        return None
    return match.group(2)


def find_camera_inputs(capture_folder: Path) -> list[CameraReplayInput]:
    found: list[CameraReplayInput] = []
    video_paths = sorted(
        path for path in capture_folder.iterdir() if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS
    )
    for video_path in video_paths:
        camera = detect_camera_name(video_path)
        if camera is None:
            continue

        metadata_path = video_path.with_name(f"{video_path.stem}_metadata.mcap")
        metadata = metadata_path if metadata_path.exists() else None
        found.append(
            CameraReplayInput(
                stream_id=video_path.stem,
                camera=camera,
                video=video_path,
                metadata=metadata,
            )
        )

    if not found:
        raise FileNotFoundError(f"No camera videos found in {capture_folder}")

    cameras = list(found)
    cameras.sort(key=lambda item: camera_sort_key(item.camera))
    return cameras


def resolve_points_mcap(capture_folder: Path, explicit_path: Path | None) -> Path:
    if explicit_path is not None:
        path = explicit_path if explicit_path.is_absolute() else capture_folder / explicit_path
        if not path.exists():
            raise FileNotFoundError(f"Points MCAP does not exist: {path}")
        return path

    matches = sorted(capture_folder.glob("*ARUCO_POINTS.mcap"))
    if len(matches) == 0:
        raise FileNotFoundError(f"No *ARUCO_POINTS.mcap file found in {capture_folder}")

    reference_matches = [path for path in matches if "reference" in path.stem.lower()]
    if len(reference_matches) == 1:
        return reference_matches[0]
    if len(reference_matches) > 1:
        raise RuntimeError(
            "Multiple reference ARUCO points files found. Pass one explicitly via --points-mcap.\n"
            + "\n".join(str(path) for path in reference_matches)
        )

    if len(matches) == 1:
        return matches[0]

    raise RuntimeError(
        "Multiple ARUCO points files found. Pass one explicitly via --points-mcap.\n"
        + "\n".join(str(path) for path in matches)
    )


def resolve_target_points_mcap(
    capture_folder: Path, reference_points_mcap: Path, explicit_path: Path | None
) -> Path:
    if explicit_path is not None:
        path = explicit_path if explicit_path.is_absolute() else capture_folder / explicit_path
        if not path.exists():
            raise FileNotFoundError(f"Target points MCAP does not exist: {path}")
        return path

    all_matches = sorted(capture_folder.glob("*ARUCO_POINTS.mcap"))
    matches = [path for path in all_matches if path.resolve() != reference_points_mcap.resolve()]
    if len(matches) == 0:
        raise FileNotFoundError(
            f"Could not find a target *ARUCO_POINTS.mcap in {capture_folder} "
            f"(reference is {reference_points_mcap.name})"
        )

    by_name = [path for path in matches if "target" in path.stem.lower()]
    if len(by_name) == 1:
        return by_name[0]
    if len(by_name) > 1:
        raise RuntimeError(
            "Multiple target ARUCO points files found. Pass one explicitly via --target-points-mcap.\n"
            + "\n".join(str(path) for path in by_name)
        )

    by_metadata: list[Path] = []
    for path in matches:
        point_type = str(load_camera_config(path).get("point_type", "")).strip().lower()
        if point_type == "target":
            by_metadata.append(path)

    if len(by_metadata) == 1:
        return by_metadata[0]
    if len(by_metadata) > 1:
        raise RuntimeError(
            "Multiple target ARUCO points files found by metadata. "
            "Pass one explicitly via --target-points-mcap.\n"
            + "\n".join(str(path) for path in by_metadata)
        )

    if len(matches) == 1:
        return matches[0]

    raise RuntimeError(
        "Could not resolve unique target ARUCO points file. "
        "Pass one explicitly via --target-points-mcap.\n"
        + "\n".join(str(path) for path in matches)
    )


def resolve_depth_mcap(capture_folder: Path, explicit_path: Path | None) -> Path | None:
    if explicit_path is not None:
        path = explicit_path if explicit_path.is_absolute() else capture_folder / explicit_path
        if not path.exists():
            raise FileNotFoundError(f"Depth MCAP does not exist: {path}")
        return path

    aligned_depth = sorted(capture_folder.glob("*_ALIGNED_DEPTH.mcap"))
    if len(aligned_depth) == 1:
        return aligned_depth[0]
    if len(aligned_depth) > 1:
        raise RuntimeError(
            "Multiple *_ALIGNED_DEPTH.mcap files found. Pass one explicitly via --depth-mcap.\n"
            + "\n".join(str(path) for path in aligned_depth)
        )

    aligned_depth_legacy = sorted(capture_folder.glob("*_DEPTH_ALIGNED.mcap"))
    if len(aligned_depth_legacy) == 1:
        return aligned_depth_legacy[0]
    if len(aligned_depth_legacy) > 1:
        raise RuntimeError(
            "Multiple *_DEPTH_ALIGNED.mcap files found. Pass one explicitly via --depth-mcap.\n"
            + "\n".join(str(path) for path in aligned_depth_legacy)
        )

    depth_matches = sorted(capture_folder.glob("*_DEPTH.mcap"))
    if len(depth_matches) == 1:
        return depth_matches[0]
    if len(depth_matches) > 1:
        raise RuntimeError(
            "Multiple *_DEPTH.mcap files found. Pass one explicitly via --depth-mcap.\n"
            + "\n".join(str(path) for path in depth_matches)
        )

    return None


def load_camera_config(points_mcap_path: Path) -> dict[str, Any]:
    with points_mcap_path.open("rb") as f:
        reader = make_reader(f)
        for metadata in reader.iter_metadata():
            if metadata.name != "camera_config":
                continue
            raw = metadata.metadata.get("json")
            if raw is None:
                continue
            if isinstance(raw, bytes):
                raw = raw.decode("utf-8")
            return json.loads(raw)
    return {}


def load_camera_socket(points_mcap_path: Path) -> str | None:
    config = load_camera_config(points_mcap_path)
    socket = str(config.get("socket", "")).strip()
    return socket or None


def resolve_replay_input_for_role(
    camera_inputs: list[CameraReplayInput],
    camera_socket: str,
    rectified: bool,
    role: str,
) -> CameraReplayInput:
    role_lower = role.strip().lower()
    role_candidates = [item for item in camera_inputs if role_lower in item.video.stem.lower()]

    socket_candidates: list[str] = []
    if role_lower == "target" and rectified and not camera_socket.endswith("_R"):
        socket_candidates.append(f"{camera_socket}_R")
    socket_candidates.append(camera_socket)
    if camera_socket.endswith("_R"):
        socket_candidates.append(camera_socket[:-2])

    candidates: list[CameraReplayInput] = []
    for socket_name in socket_candidates:
        socket_matches = [item for item in role_candidates if item.camera == socket_name]
        if socket_matches:
            candidates.extend(socket_matches)
            break

    if not candidates:
        for socket_name in socket_candidates:
            socket_matches = [item for item in camera_inputs if item.camera == socket_name]
            if socket_matches:
                candidates.extend(socket_matches)
                break

    if role_lower == "target":
        if rectified:
            preferred = [item for item in candidates if "_R" in item.video.stem]
            if preferred:
                candidates = preferred
        else:
            preferred = [item for item in candidates if "_R" not in item.video.stem]
            if preferred:
                candidates = preferred

    if len(candidates) == 1:
        return candidates[0]

    if not candidates:
        raise RuntimeError(
            f"Could not resolve replay {role_lower} stream for camera {camera_socket}. "
            f"Available videos: {[item.video.name for item in camera_inputs]}"
        )

    raise RuntimeError(
        f"Ambiguous replay {role_lower} stream for camera {camera_socket}. "
        f"Candidates: {[item.video.name for item in candidates]}"
    )


def format_size_from_config(config: dict[str, Any]) -> str:
    size = config.get("size")
    if not isinstance(size, dict):
        return "unknown"
    width = size.get("width")
    height = size.get("height")
    if width is None or height is None:
        return "unknown"
    return f"{width}x{height}"


def print_camera_characteristics(
    title: str,
    camera_socket: str,
    replay_camera: str,
    config: dict[str, Any],
    replay_video_name: str,
) -> None:
    print(f"{title} characteristics:")
    print(f"  Camera: {camera_socket} (replay: {replay_camera}, video: {replay_video_name})")
    print(f"  Size: {format_size_from_config(config)}")
    print(f"  Resize mode: {config.get('resize_mode', 'unknown')}")
    print(f"  enableUndistortion: {bool(config.get('undistortion_enabled', False))}")
    print(f"  Rectification: {bool(config.get('rectified', False))}")


def load_points_frames(points_mcap_path: Path, camera_name: str) -> list[PointsFrame]:
    frames: list[PointsFrame] = []
    with points_mcap_path.open("rb") as f:
        reader = make_reader(f)
        for _, channel, message in reader.iter_messages():
            if channel.topic not in {"reference_points", "target_points"}:
                continue

            payload = json.loads(bytes(message.data).decode("utf-8"))
            points = payload.get("points")
            if points is None:
                continue

            timestamp_us = payload.get("timestamp_us")
            if timestamp_us is not None:
                timestamp_device_ns = int(timestamp_us) * 1000
            else:
                publish_time = int(getattr(message, "publish_time", 0))
                log_time = int(getattr(message, "log_time", 0))
                timestamp_device_ns = publish_time if publish_time > 0 else log_time

            sequence_num = int(getattr(message, "sequence", len(frames)))
            frames.append(
                PointsFrame(
                    reference_camera=camera_name,
                    sequence_num=sequence_num,
                    timestamp_device_ns=timestamp_device_ns,
                    points=list(points),
                )
            )

    if not frames:
        raise RuntimeError(f"No points frames found in {points_mcap_path}")
    return frames


def to_pixel_coordinate(value: Any, size: int) -> float:
    value = float(value)
    if 0.0 <= value <= 1.0:
        return value * float(size)
    return value


def extract_point_id(point: dict[str, Any]) -> tuple[Any, ...] | None:
    charuco_corner_id = point.get("charuco_corner_id")
    if charuco_corner_id is not None:
        return ("charuco_corner_id", int(charuco_corner_id))

    aruco_id = point.get("aruco_id")
    marker_corner_index = point.get("marker_corner_index")
    if aruco_id is not None and marker_corner_index is not None:
        return ("aruco_corner", int(aruco_id), int(marker_corner_index))

    return None


def sample_depth_mm(depth_frame: np.ndarray, x: int, y: int, radius: int = 3) -> int | None:
    height, width = depth_frame.shape[:2]
    if x < 0 or y < 0 or x >= width or y >= height:
        return None

    x0 = max(0, x - radius)
    x1 = min(width - 1, x + radius)
    y0 = max(0, y - radius)
    y1 = min(height - 1, y + radius)

    roi = depth_frame[y0 : y1 + 1, x0 : x1 + 1]
    valid = roi[roi > 0]
    if valid.size == 0:
        return None
    return int(np.median(valid))


def match_target_points_frame(
    reference_points_frame: PointsFrame,
    target_points_by_sequence: dict[int, PointsFrame],
    target_points_frames: list[PointsFrame],
) -> PointsFrame | None:
    by_sequence = target_points_by_sequence.get(reference_points_frame.sequence_num)
    if by_sequence is not None:
        return by_sequence
    if not target_points_frames:
        return None
    return min(
        target_points_frames,
        key=lambda frame: abs(frame.timestamp_device_ns - reference_points_frame.timestamp_device_ns),
    )


def to_reprojected_from_aruco_points(
    reference_transform: dai.ImgTransformation,
    target_transform: dai.ImgTransformation,
    aruco_points: list[dict[str, Any]],
    ref_width: int,
    ref_height: int,
    depth_frame_mm: np.ndarray | None,
) -> tuple[list[tuple[float, float]], dict[tuple[Any, ...], tuple[float, float]]]:

    projected_points: list[tuple[float, float]] = []
    projected_points_by_id: dict[tuple[Any, ...], tuple[float, float]] = {}
    
    # testPont = dai.Point2f(510, 529, False)
    # depth_mm = 1230
    # print("test point: (", testPont.x, ",", testPont.y, ") with depth ", depth_mm, "mm")
    # projected_test = reference_transform.projectPoint(target_transform, testPont, depth_mm)
    # print(f"Projected test point: ({projected_test.x}, {projected_test.y})")
    # projected_points.append((float(projected_test.x), float(projected_test.y)))
    
    for point in aruco_points:
        if point.get("valid") is False:
            continue

        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue

        x_px = to_pixel_coordinate(px, ref_width)
        y_px = to_pixel_coordinate(py, ref_height)
        point2f: dai.Point2f = dai.Point2f(x_px, y_px, False)

        depth_mm_value: float | None = None
        depth_mm = point.get("depth_mm")
        if depth_mm is not None:
            depth_candidate = float(depth_mm)
            if math.isfinite(depth_candidate) and depth_candidate > 0.0:
                depth_mm_value = depth_candidate

        if depth_mm_value is None and depth_frame_mm is not None:
            sampled = sample_depth_mm(depth_frame_mm, int(round(x_px)), int(round(y_px)), radius=5)
            if sampled is not None:
                depth_mm_value = float(sampled)

        if depth_mm_value is not None:
            # print("CAM_B distortion coefficients:", target_transform.getDistortionCoefficients())
            # print(f"Projecting with depth {depth_mm_value}mm at pixel ({x_px}, {y_px})")
            # print(f"target transform size: {target_transform.getSize()}, source size: {reference_transform.getSourceSize()}")
            ## TODO(matic)
            projected = reference_transform.projectPoint(target_transform, point2f, depth_mm_value)
            # print(f"Projected point: ({projected.x}, {projected.y})")
            projected_point = (float(projected.x), float(projected.y))
            projected_points.append(projected_point)
            point_id = extract_point_id(point)
            if point_id is not None:
                if point_id in projected_points_by_id:
                    print(f"Warning: duplicate projected point id {point_id}, keeping first occurrence.")
                else:
                    projected_points_by_id[point_id] = projected_point
            continue

        # projected = reference_transform.remapPointTo(target_transform, dai.Point2f(float(x_px), float(y_px)))
        # projected_points.append((float(projected.x), float(projected.y)))

    return projected_points, projected_points_by_id
    # std::cout << "Depth: " << depth << "mm, Point in source frame (cm): (" << x_cm << ", " << y_cm << ", " << z_cm << ")" << std::endl;


def evaluate_self_roundtrip_error(
    transform: dai.ImgTransformation,
    points: list[dict[str, Any]],
    frame_width: int,
    frame_height: int,
    depth_frame_mm: np.ndarray | None,
) -> tuple[list[float], int, int, dict[str, float] | None]:
    errors_px: list[float] = []
    attempted = 0
    failed = 0

    for point in points:
        if point.get("valid") is False:
            continue

        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue

        x_px = to_pixel_coordinate(px, frame_width)
        y_px = to_pixel_coordinate(py, frame_height)
        if not (math.isfinite(x_px) and math.isfinite(y_px)):
            continue

        depth_mm_value: float | None = None
        depth_mm = point.get("depth_mm")
        if depth_mm is not None:
            depth_candidate = float(depth_mm)
            if math.isfinite(depth_candidate) and depth_candidate > 0.0:
                depth_mm_value = depth_candidate

        if depth_mm_value is None and depth_frame_mm is not None:
            sampled = sample_depth_mm(depth_frame_mm, int(round(x_px)), int(round(y_px)), radius=5)
            if sampled is not None:
                depth_mm_value = float(sampled)

        if depth_mm_value is None:
            continue

        attempted += 1
        source_point = dai.Point2f(float(x_px), float(y_px), False)
        source_point_copy = dai.Point2f(float(x_px), float(y_px), False)
        try:
            roundtrip = transform.projectPoint(transform, source_point_copy, float(depth_mm_value))
        except Exception:
            failed += 1
            continue

        rx = float(roundtrip.x)
        ry = float(roundtrip.y)
        if not (math.isfinite(rx) and math.isfinite(ry)):
            failed += 1
            continue

        errors_px.append(float(math.hypot(rx - source_point.x, ry - source_point.y)))

    stats: dict[str, float] | None = None
    if errors_px:
        values = np.asarray(errors_px, dtype=np.float64)
        stats = {
            "mean_px": float(np.mean(values)),
            "median_px": float(np.median(values)),
            "p95_px": float(np.percentile(values, 95.0)),
            "max_px": float(np.max(values)),
        }

    return errors_px, attempted, failed, stats

# def to_reprojected_from_aruco_points(
#     reference_transform: dai.ImgTransformation,
#     target_transform: dai.ImgTransformation,
#     aruco_points: list[dict[str, Any]],
#     ref_width: int,
#     ref_height: int,
#     depth_frame_mm: np.ndarray | None,
# ) -> list[tuple[float, float]]:
#     intr = np.asarray(reference_transform.getIntrinsicMatrix(), dtype=np.float64)
#     fx = float(intr[0, 0])
#     fy = float(intr[1, 1])
#     cx = float(intr[0, 2])
#     cy = float(intr[1, 2])

#     projected_points: list[tuple[float, float]] = []
#     for point in aruco_points:
#         px = point.get("pixel_x")
#         py = point.get("pixel_y")
#         if px is None or py is None:
#             continue

#         x_px = to_pixel_coordinate(px, ref_width)
#         y_px = to_pixel_coordinate(py, ref_height)
#         # point: dai.Point2f = dai.Point2f(x_px, y_px, False)

#         depth_mm_value: float | None = None
#         depth_mm = point.get("depth_mm")
#         if depth_mm is not None:
#             depth_candidate = float(depth_mm)
#             if math.isfinite(depth_candidate) and depth_candidate > 0.0:
#                 depth_mm_value = depth_candidate

#         if depth_mm_value is None and depth_frame_mm is not None:
#             sampled = sample_depth_mm(depth_frame_mm, int(round(x_px)), int(round(y_px)), radius=1)
#             if sampled is not None:
#                 depth_mm_value = float(sampled)

#         if depth_mm_value is not None:
#             z_cm = depth_mm_value / 10.0
#             x_cm = ((x_px - cx) * depth_mm_value / fx) / 10.0
#             y_cm = ((y_px - cy) * depth_mm_value / fy) / 10.0
#             source_3d_cm = dai.Point3f(float(x_cm), float(y_cm), float(z_cm))
#             projected = reference_transform.project3DPointTo(target_transform, source_3d_cm)
#             projected_points.append((float(projected.x), float(projected.y)))
#             continue

#         projected = reference_transform.remapPointTo(target_transform, dai.Point2f(float(x_px), float(y_px)))
#         projected_points.append((float(projected.x), float(projected.y)))

#     return projected_points


def compare_points_by_ids(
    projected_points_by_id: dict[tuple[Any, ...], tuple[float, float]],
    target_points: list[dict[str, Any]],
    target_width: int,
    target_height: int,
) -> tuple[list[float], int, int, int, dict[str, float] | None]:
    gt_by_id: dict[tuple[Any, ...], tuple[float, float]] = {}
    for point in target_points:
        point_id = extract_point_id(point)
        if point_id is None:
            continue

        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue
        x = to_pixel_coordinate(px, target_width)
        y = to_pixel_coordinate(py, target_height)
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if point_id in gt_by_id:
            print(f"Warning: duplicate target point id {point_id}, keeping first occurrence.")
            continue
        gt_by_id[point_id] = (x, y)

    common_ids = set(projected_points_by_id.keys()) & set(gt_by_id.keys())
    errors_px: list[float] = []
    dx_px: list[float] = []
    dy_px: list[float] = []
    for point_id in common_ids:
        gx, gy = gt_by_id[point_id]
        px, py = projected_points_by_id[point_id]
        if not (math.isfinite(px) and math.isfinite(py)):
            continue

        dx = float(px - gx)
        dy = float(py - gy)
        dx_px.append(dx)
        dy_px.append(dy)
        # print("Projected point id:", point_id, "GT (px): (", gx, ",", gy, ") vs Projected (px): (", px, ",", py, ") with error (px): ", math.hypot(dx, dy))
        errors_px.append(float(math.hypot(px - gx, py - gy)))

    residual_stats: dict[str, float] | None = None
    if dx_px and dy_px:
        residual_stats = {
            "mean_dx_px": float(np.mean(dx_px)),
            "std_dx_px": float(np.std(dx_px)),
            "mean_dy_px": float(np.mean(dy_px)),
            "std_dy_px": float(np.std(dy_px)),
        }

    return (
        errors_px,
        len(errors_px),
        len(projected_points_by_id),
        len(gt_by_id),
        residual_stats,
    )


def extract_pixel_points(points: list[dict[str, Any]], frame_width: int, frame_height: int) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for point in points:
        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue
        x = to_pixel_coordinate(px, frame_width)
        y = to_pixel_coordinate(py, frame_height)
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        out.append((x, y))
    return out


def extract_pixel_points_with_validity(
    points: list[dict[str, Any]], frame_width: int, frame_height: int
) -> list[tuple[float, float, bool]]:
    out: list[tuple[float, float, bool]] = []
    for point in points:
        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue
        x = to_pixel_coordinate(px, frame_width)
        y = to_pixel_coordinate(py, frame_height)
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        valid_raw = point.get("valid")
        valid = bool(valid_raw) if valid_raw is not None else True
        out.append((x, y, valid))
    return out


def draw_points(frame: Any, points: list[tuple[float, float]], color: tuple[int, int, int]) -> None:
    for x, y in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        cv2.circle(frame, (int(round(x)), int(round(y))), POINT_RADIUS, color, -1, cv2.LINE_AA)


def draw_points_with_validity(
    frame: Any,
    points: list[tuple[float, float, bool]],
    valid_color: tuple[int, int, int],
    invalid_color: tuple[int, int, int],
) -> None:
    for x, y, valid in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        color = valid_color if valid else invalid_color
        cv2.circle(frame, (int(round(x)), int(round(y))), POINT_RADIUS, color, -1, cv2.LINE_AA)


def extract_reference_depth_labels(
    points: list[dict[str, Any]],
    frame_width: int,
    frame_height: int,
    depth_frame_mm: np.ndarray | None,
    point_stride: int = DEPTH_LABEL_POINT_STRIDE,
) -> list[tuple[int, int, str]]:
    labels: list[tuple[int, int, str]] = []
    stride = max(1, int(point_stride))
    for idx, point in enumerate(points):
        if idx % stride != 0:
            continue

        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue

        x = to_pixel_coordinate(px, frame_width)
        y = to_pixel_coordinate(py, frame_height)
        if not (math.isfinite(x) and math.isfinite(y)):
            continue

        depth_mm_value: int | None = None
        depth_mm_raw = point.get("depth_mm")
        if depth_mm_raw is not None:
            depth_candidate = float(depth_mm_raw)
            if math.isfinite(depth_candidate) and depth_candidate > 0.0:
                depth_mm_value = int(round(depth_candidate))

        if depth_mm_value is None and depth_frame_mm is not None:
            depth_mm_value = sample_depth_mm(depth_frame_mm, int(round(x)), int(round(y)), radius=1)

        label = f"{depth_mm_value}mm" if depth_mm_value is not None else "n/a"
        labels.append((int(round(x)), int(round(y)), label))
    return labels


def draw_depth_labels(
    frame: np.ndarray,
    labels: list[tuple[int, int, str]],
    text_color: tuple[int, int, int] = (255, 255, 255),
) -> None:
    for x, y, label in labels:
        cv2.circle(frame, (x, y), 1, text_color, -1, cv2.LINE_AA)
        anchor = (x + 3, y - 3)
        cv2.putText(
            frame,
            label,
            anchor,
            cv2.FONT_HERSHEY_PLAIN,
            0.7,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            label,
            anchor,
            cv2.FONT_HERSHEY_PLAIN,
            0.7,
            text_color,
            1,
            cv2.LINE_AA,
        )


def add_frame_title(frame: np.ndarray, title: str) -> None:
    cv2.putText(
        frame,
        title,
        (12, frame.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )


def pad_frame_to_size(frame: np.ndarray, target_height: int, target_width: int) -> np.ndarray:
    height, width = frame.shape[:2]
    pad_bottom = max(0, target_height - height)
    pad_right = max(0, target_width - width)
    if pad_bottom == 0 and pad_right == 0:
        return frame
    return cv2.copyMakeBorder(
        frame,
        0,
        pad_bottom,
        0,
        pad_right,
        borderType=cv2.BORDER_CONSTANT,
        value=(0, 0, 0),
    )

def concatenate_frames_grid_2x2(
    top_left: np.ndarray,
    top_right: np.ndarray,
    bottom_left: np.ndarray,
    bottom_right: np.ndarray,
) -> np.ndarray:
    top_height = max(top_left.shape[0], top_right.shape[0])
    top_width = max(top_left.shape[1], top_right.shape[1])
    top_left_padded = pad_frame_to_size(top_left, top_height, top_width)
    top_right_padded = pad_frame_to_size(top_right, top_height, top_width)

    bottom_height = max(bottom_left.shape[0], bottom_right.shape[0])
    # Requirement: bottom tile width matches the largest ref/target frame width.
    bottom_width = top_width
    bottom_left_padded = pad_frame_to_size(bottom_left, bottom_height, bottom_width)
    bottom_right_padded = pad_frame_to_size(bottom_right, bottom_height, bottom_width)

    top_row = cv2.hconcat([top_left_padded, top_right_padded])
    bottom_row = cv2.hconcat([bottom_left_padded, bottom_right_padded])
    return cv2.vconcat([top_row, bottom_row])


def render_error_histogram(
    errors_px: list[float],
    title: str = "ID-matched reprojection error histogram",
    bins: int = 30,
    max_error_px: float = 20.0,
    width: int = 760,
    height: int = 280,
) -> np.ndarray:
    canvas = np.full((height, width, 3), 24, dtype=np.uint8)
    cv2.putText(
        canvas,
        title,
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (240, 240, 240),
        2,
        cv2.LINE_AA,
    )

    left = 48
    right = width - 20
    top = 42
    bottom = height - 44
    cv2.rectangle(canvas, (left, top), (right, bottom), (120, 120, 120), 1)

    if len(errors_px) == 0:
        cv2.putText(
            canvas,
            "No ID-matched points yet.",
            (left + 10, (top + bottom) // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (200, 200, 200),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"x-range: 0 .. {max_error_px:.1f}px",
            (left, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )
        return canvas

    values = np.asarray(errors_px, dtype=np.float64)
    clipped = np.clip(values, 0.0, max_error_px)
    hist, _ = np.histogram(clipped, bins=bins, range=(0.0, max_error_px))

    plot_width = max(1, right - left - 2)
    plot_height = max(1, bottom - top - 2)
    max_count = max(1, int(np.max(hist)))
    bin_width = plot_width / float(bins)

    for idx, count in enumerate(hist.tolist()):
        x0 = int(left + 1 + idx * bin_width)
        x1 = int(left + 1 + (idx + 1) * bin_width) - 1
        x1 = max(x1, x0)
        bar_h = int((count / max_count) * plot_height)
        y0 = bottom - 1 - bar_h
        cv2.rectangle(canvas, (x0, y0), (x1, bottom - 1), (90, 170, 255), -1)

    mean_err = float(np.mean(values))
    median_err = float(np.median(values))
    p95_err = float(np.percentile(values, 95))
    cv2.putText(
        canvas,
        f"n={len(values)} mean={mean_err:.2f}px median={median_err:.2f}px p95={p95_err:.2f}px",
        (12, height - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (220, 220, 220),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"x-range: 0 .. {max_error_px:.1f}px",
        (width - 170, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (190, 190, 190),
        1,
        cv2.LINE_AA,
    )
    return canvas


def format_overlay_timestamp(ts: Any) -> str:
    if ts is None:
        return "n/a"
    if hasattr(ts, "total_seconds"):
        return f"{ts.total_seconds():.6f}s"
    return str(ts)


def to_ns(ts: Any) -> int:
    if ts is None:
        return 0
    if isinstance(ts, int):
        return ts
    if isinstance(ts, float):
        return int(ts * 1_000_000_000)
    if hasattr(ts, "total_seconds"):
        return int(ts.total_seconds() * 1_000_000_000)
    return 0


def get_nearest_imgframe_for_reference_ts(
    queue: Any,
    reference_device_ts_ns: int,
    pending: dai.ImgFrame | None,
) -> tuple[dai.ImgFrame | None, dai.ImgFrame | None]:
    prev = pending
    while True:
        try:
            candidate = queue.get()
        except dai.MessageQueue.QueueException:
            # Replay ended; return the closest previously seen frame if available.
            return prev, None

        if not isinstance(candidate, dai.ImgFrame):
            continue

        candidate_ts_ns = to_ns(candidate.getTimestampDevice())
        if candidate_ts_ns >= reference_device_ts_ns:
            if prev is None:
                return candidate, None
            prev_ts_ns = to_ns(prev.getTimestampDevice())
            if abs(prev_ts_ns - reference_device_ts_ns) <= abs(candidate_ts_ns - reference_device_ts_ns):
                return prev, candidate
            return candidate, None

        prev = candidate


def wait_for_step_key() -> bool:
    while True:
        key = cv2.waitKey(0) & 0xFF
        if key in (ord("l"), ord("L")):
            return True
        if key in (ord("q"), 27):
            return False


def create_replay_queues(
    pipeline: dai.Pipeline, camera_inputs: list[CameraReplayInput]
) -> dict[str, Any]:
    queues: dict[str, Any] = {}
    for cam in camera_inputs:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(cam.video)
        if cam.metadata is not None:
            replay.setReplayMetadataFile(cam.metadata)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
        replay.setLoop(False)
        queues[cam.stream_id] = replay.out.createOutputQueue(maxSize=1, blocking=True)
    return queues


def create_depth_replay_queue(pipeline: dai.Pipeline, depth_mcap_path: Path) -> Any:
    replay = pipeline.create(dai.node.ReplayMetadataOnly)
    replay.setReplayFile(depth_mcap_path)
    replay.setLoop(False)
    return replay.out.createOutputQueue(maxSize=1, blocking=True)


def depth_to_colormap(depth_mm: np.ndarray) -> np.ndarray:
    if depth_mm.ndim != 2:
        raise ValueError(f"Expected single-channel depth frame, got shape={depth_mm.shape}")

    vis = np.zeros(depth_mm.shape, dtype=np.uint8)
    valid_mask = depth_mm > 0
    valid_values = depth_mm[valid_mask]
    if valid_values.size == 0:
        return cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    depth_min = float(np.percentile(valid_values, 2))
    depth_max = float(np.percentile(valid_values, 98))
    if depth_max <= depth_min:
        depth_min = float(valid_values.min())
        depth_max = float(valid_values.max())

    if depth_max <= depth_min:
        vis[valid_mask] = 255
    else:
        scaled = np.clip((depth_mm.astype(np.float32) - depth_min) * (255.0 / (depth_max - depth_min)), 0.0, 255.0)
        vis[valid_mask] = scaled[valid_mask].astype(np.uint8)

    colored = cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)
    colored[~valid_mask] = 0
    return colored


def overlay_depth_on_frame(frame_bgr: np.ndarray, depth_mm: np.ndarray, alpha: float = 0.7) -> np.ndarray:
    frame_h, frame_w = frame_bgr.shape[:2]
    if depth_mm.shape[:2] != (frame_h, frame_w):
        depth_mm = cv2.resize(depth_mm, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)

    depth_color = depth_to_colormap(depth_mm)
    blended = cv2.addWeighted(frame_bgr, 1.0 - alpha, depth_color, alpha, 0.0)

    valid_mask = depth_mm > 0
    out = frame_bgr.copy()
    out[valid_mask] = blended[valid_mask]
    return out


def printTransformation(label, transformation: dai.ImgTransformation):
    print(f"{label} transformation:")
    
    extrinsics: dai.Extrinsics = transformation.getExtrinsics()
    rotation = extrinsics.rotationMatrix
    translation = extrinsics.getTranslationVector()
    unit = extrinsics.lengthUnit
    
    extrinsics_matrix = [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0, 0, 0, 1]
    ]
    
    
    print("Extrinsics matrix:")
    for row in extrinsics_matrix:
        print("    ",row)
    print(f"Units: {unit}")
    print("toCameraSocket: ", extrinsics.toCameraSocket)
    print("\n")
    
    transformationMatrix = transformation.getTransformationMatrix()
    print(f"Transformation matrix")
    for row in transformationMatrix:
        print("    ", row)
    print("\n")
    
    sourceIntrinsics = transformation.getSourceIntrinsicMatrix()
    print(f"Source intrinsics matrix")
    for row in sourceIntrinsics:
        print("    ", row)
    print("\n")
    
    transformationMatrix = transformation.getTransformationMatrix()
    
    intrinsics = transformation.getIntrinsicMatrix()
    print(f"Intrinsics matrix = sourceIntrinsicMatrix @ TransformationMatrix)")
    for row in intrinsics:
        print("    ", row)
    print("\n")
    
    print("Source matrix inverse:")
    sourceIntrinsicsInv = transformation.getSourceIntrinsicMatrixInv()
    for row in sourceIntrinsicsInv:
        print("    ", row)
    print("\n")
    
    print("intrinsics inverse:")
    intrinsicsInv = transformation.getIntrinsicMatrixInv()
    for row in intrinsicsInv:
        print("    ", row)
    print("\n")
    
    print("Transformation matrix inverse:")
    transformationMatrixInv = transformation.getTransformationMatrixInv()
    for row in transformationMatrixInv:
        print("    ", row)
    print("\n")
    
    print(f"Distortion coefficients: {transformation.getDistortionCoefficients()}")
    
    
    source_width, source_height = transformation.getSourceSize()
    width, height = transformation.getSize()
    print(f"Source size: {source_width}x{source_height}")
    print(f"Size: {width}x{height}")
    
    
    print("==================================================================================================================\n")




def main() -> None:
    if os.environ.get("DISPLAY", "") == "":
        raise RuntimeError("DISPLAY is not set. This script uses cv2.imshow and must run in a GUI session.")

    args = parse_args()
    capture_folder = Path(args.capture_folder).expanduser().resolve()
    if not capture_folder.exists():
        raise FileNotFoundError(f"Capture folder does not exist: {capture_folder}")

    camera_inputs = find_camera_inputs(capture_folder)
    reference_points_mcap = resolve_points_mcap(capture_folder, args.points_mcap)
    target_points_mcap = resolve_target_points_mcap(capture_folder, reference_points_mcap, args.target_points_mcap)
    depth_mcap = resolve_depth_mcap(capture_folder, args.depth_mcap)

    reference_config = load_camera_config(reference_points_mcap)
    target_config = load_camera_config(target_points_mcap)

    reference_camera_socket = str(reference_config.get("socket", "")).strip()
    if not reference_camera_socket:
        reference_camera_socket = detect_camera_name(reference_points_mcap) or ""
    if not reference_camera_socket:
        raise RuntimeError(
            f"Could not determine reference camera from points file metadata: {reference_points_mcap}. "
            "Pass a file created by new_save_aruco.py."
        )

    target_camera_socket = str(target_config.get("socket", "")).strip()
    if not target_camera_socket:
        target_camera_socket = detect_camera_name(target_points_mcap) or ""
    if not target_camera_socket:
        raise RuntimeError(
            f"Could not determine target camera from points file metadata: {target_points_mcap}. "
            "Pass a file created by new_save_aruco.py."
        )

    reference_points_frames = load_points_frames(reference_points_mcap, reference_camera_socket)
    target_points_frames = load_points_frames(target_points_mcap, target_camera_socket)
    target_points_by_sequence = {frame.sequence_num: frame for frame in target_points_frames}

    reference_replay_input = resolve_replay_input_for_role(
        camera_inputs,
        camera_socket=reference_camera_socket,
        rectified=bool(reference_config.get("rectified", False)),
        role="reference",
    )
    target_replay_input = resolve_replay_input_for_role(
        camera_inputs,
        camera_socket=target_camera_socket,
        rectified=bool(target_config.get("rectified", False)),
        role="target",
    )

    print(f"Capture folder: {capture_folder}")
    print(f"Reference points file: {reference_points_mcap}")
    print(f"Target points file: {target_points_mcap}")
    if depth_mcap is None:
        print("Depth file: not found (depth visualization disabled)")
    else:
        print(f"Depth file: {depth_mcap}")

    valid_depth = 0
    for p in reference_points_frames[0].points:
        depth_mm = p.get("depth_mm")
        if depth_mm is None:
            continue
        depth_value = float(depth_mm)
        if math.isfinite(depth_value) and depth_value > 0.0:
            valid_depth += 1
    # print(
    #     "Spatial points usable for 3D reprojection in first frame: "
    #     f"{valid_depth}/{len(reference_points_frames[0].points)}"
    # )
    print_camera_characteristics(
        "Reference",
        reference_camera_socket,
        reference_replay_input.camera,
        reference_config,
        reference_replay_input.video.name,
    )
    print_camera_characteristics(
        "Target",
        target_camera_socket,
        target_replay_input.camera,
        target_config,
        target_replay_input.video.name,
    )
    print("Controls: press 'l' for next frame, 'q' or ESC to quit.")

    with dai.Pipeline() as pipeline:
        queues = create_replay_queues(pipeline, camera_inputs)
        depth_queue = create_depth_replay_queue(pipeline, depth_mcap) if depth_mcap is not None else None
        pipeline.start()
        comparison_video_path = capture_folder / "comparison.mp4"
        comparison_video_fps = 10
        comparison_writer: cv2.VideoWriter | None = None
        all_id_errors_px: list[float] = []
        all_roundtrip_errors_px: list[float] = []
        pending_depth: dai.ImgFrame | None = None
        pending_by_stream: dict[str, dai.ImgFrame | None] = {cam.stream_id: None for cam in camera_inputs}

        try:
            for frame_idx, reference_points_frame in enumerate(reference_points_frames):
                assert reference_points_frame.reference_camera == reference_camera_socket, (
                    f"Reference camera changed in points file at frame {frame_idx}: "
                    f"{reference_points_frame.reference_camera} != {reference_camera_socket}"
                )
                target_points_frame = match_target_points_frame(
                    reference_points_frame, target_points_by_sequence, target_points_frames
                )
                if target_points_frame is None:
                    print(f"No target points frame available at frame {frame_idx}.")
                    return

                frames: dict[str, dai.ImgFrame] = {}
                if reference_points_frame.timestamp_device_ns > 0:
                    for cam in camera_inputs:
                        img, pending_by_stream[cam.stream_id] = get_nearest_imgframe_for_reference_ts(
                            queues[cam.stream_id],
                            reference_points_frame.timestamp_device_ns,
                            pending_by_stream[cam.stream_id],
                        )
                        if img is None:
                            print(f"Replay ended at frame {frame_idx}.")
                            return
                        frames[cam.stream_id] = img
                else:
                    for cam in camera_inputs:
                        try:
                            img = queues[cam.stream_id].get()
                        except dai.MessageQueue.QueueException:
                            print(f"Replay ended at frame {frame_idx}.")
                            return
                        assert isinstance(img, dai.ImgFrame)
                        frames[cam.stream_id] = img

                reference_msg = frames[reference_replay_input.stream_id]
                target_msg = frames[target_replay_input.stream_id]
                ref_transform = reference_msg.getTransformation()
                target_transform = target_msg.getTransformation()
                # print ref and target transform details for debugging
                printTransformation("Target", target_transform)
                printTransformation("Reference", ref_transform)
                
                
                ref_w = int(reference_msg.getWidth())
                ref_h = int(reference_msg.getHeight())
                ref_device_ts_ns = to_ns(reference_msg.getTimestampDevice())

                depth_msg: dai.ImgFrame | None = None
                depth_dt_ms: float | None = None
                depth_frame_mm: np.ndarray | None = None
                if depth_queue is not None:
                    depth_msg, pending_depth = get_nearest_imgframe_for_reference_ts(
                        depth_queue, ref_device_ts_ns, pending_depth
                    )
                    if depth_msg is not None:
                        depth_dt_ms = (to_ns(depth_msg.getTimestampDevice()) - ref_device_ts_ns) / 1_000_000.0
                        depth_frame_mm = depth_msg.getFrame()

                try:
                    matrix = ref_transform.getExtrinsicsTransformationMatrixTo(target_transform)
                    tx = float(matrix[0][3])
                    ty = float(matrix[1][3])
                    tz = float(matrix[2][3])
                    # print(
                    #     f"[frame {frame_idx}] extrinsics "
                    #     f"{reference_replay_input.stream_id} -> {target_replay_input.stream_id} "
                    #     f"translation_cm=[{tx:.3f}, {ty:.3f}, {tz:.3f}]"
                    # )
                    # print("  matrix:")
                    # for row in matrix:
                    #     print("   ", " ".join(f"{float(v): .6f}" for v in row))
                except Exception as exc:
                    print(
                        f"[frame {frame_idx}] extrinsics "
                        f"{reference_replay_input.stream_id} -> {target_replay_input.stream_id}: unavailable ({exc})"
                    )

                reference_frame = reference_msg.getCvFrame().copy()
                if depth_msg is not None:
                    reference_frame = overlay_depth_on_frame(reference_frame, depth_msg.getFrame(), alpha=0.45)
                target_frame = target_msg.getCvFrame().copy()

                projected_points, projected_points_by_id = to_reprojected_from_aruco_points(
                    ref_transform,
                    target_transform,
                    reference_points_frame.points,
                    ref_w,
                    ref_h,
                    depth_frame_mm,
                )
                valid_ref_for_projection = sum(
                    1
                    for p in reference_points_frame.points
                    if p.get("valid") is not False and p.get("pixel_x") is not None and p.get("pixel_y") is not None
                )
                # print(
                #     f"[frame {frame_idx}] projection_count valid_ref={valid_ref_for_projection} projected={len(projected_points)}"
                # )
                reference_original_points = extract_pixel_points_with_validity(
                    reference_points_frame.points, ref_w, ref_h
                )
                target_gt_points = extract_pixel_points(
                    target_points_frame.points, int(target_msg.getWidth()), int(target_msg.getHeight())
                )

                (
                    frame_id_errors_px,
                    frame_id_match_count,
                    projected_points_with_ids,
                    gt_points_with_ids,
                    frame_id_residual_stats,
                ) = compare_points_by_ids(
                    projected_points_by_id=projected_points_by_id,
                    target_points=target_points_frame.points,
                    target_width=int(target_msg.getWidth()),
                    target_height=int(target_msg.getHeight()),
                )
                (
                    frame_roundtrip_errors_px,
                    frame_roundtrip_attempted,
                    frame_roundtrip_failed,
                    frame_roundtrip_stats,
                ) = evaluate_self_roundtrip_error(
                    transform=ref_transform,
                    points=reference_points_frame.points,
                    frame_width=ref_w,
                    frame_height=ref_h,
                    depth_frame_mm=depth_frame_mm,
                )
                if frame_id_errors_px:
                    all_id_errors_px.extend(frame_id_errors_px)
                if frame_roundtrip_errors_px:
                    all_roundtrip_errors_px.extend(frame_roundtrip_errors_px)
                    
                # Requested coloring on the target frame: GT in green, projected in blue.
                draw_points(target_frame, target_gt_points, (0, 255, 0))
                draw_points(target_frame, projected_points, (255, 0, 0))
                # Draw original reference points and keep non-valid points visible as red.
                draw_points_with_validity(reference_frame, reference_original_points, (255, 255, 0), (0, 0, 255))
                reference_depth_labels = extract_reference_depth_labels(
                    reference_points_frame.points,
                    ref_w,
                    ref_h,
                    depth_frame_mm,
                    point_stride=DEPTH_LABEL_POINT_STRIDE,
                )
                draw_depth_labels(reference_frame, reference_depth_labels)

                reference_host_ts = format_overlay_timestamp(reference_msg.getTimestamp())
                reference_device_ts = format_overlay_timestamp(reference_msg.getTimestampDevice())
                cv2.putText(
                    reference_frame,
                    f"{reference_replay_input.stream_id} seq={reference_msg.getSequenceNum()} idx={frame_idx}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    reference_frame,
                    f"host_ts= {reference_host_ts} dev_ts= {reference_device_ts}",
                    (12, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if reference_points_frame.timestamp_device_ns > 0:
                    reference_dt_ms = (
                        to_ns(reference_msg.getTimestampDevice()) - reference_points_frame.timestamp_device_ns
                    ) / 1_000_000.0
                    cv2.putText(
                        reference_frame,
                        f"points_dt={reference_dt_ms:+.1f}ms",
                        (12, 84),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 200, 255),
                        2,
                        cv2.LINE_AA,
                    )
                if depth_msg is not None:
                    cv2.putText(
                        reference_frame,
                        f"depth seq={depth_msg.getSequenceNum()} dt={depth_dt_ms:+.1f}ms",
                        (12, 112),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                cv2.putText(
                    reference_frame,
                    "ref points: valid cyan invalid red",
                    (12, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    reference_frame,
                    f"depth labels: every {DEPTH_LABEL_POINT_STRIDE}th point",
                    (12, 168),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if frame_roundtrip_stats is not None:
                    cv2.putText(
                        reference_frame,
                        (
                            f"ref self-rt n={len(frame_roundtrip_errors_px)}/{frame_roundtrip_attempted} "
                            f"mean={frame_roundtrip_stats['mean_px']:.2f}px "
                            f"med={frame_roundtrip_stats['median_px']:.2f}px "
                            f"p95={frame_roundtrip_stats['p95_px']:.2f}px"
                        ),
                        (12, 196),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    cv2.putText(
                        reference_frame,
                        f"ref self-rt n=0/{frame_roundtrip_attempted} failed={frame_roundtrip_failed}",
                        (12, 196),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )

                target_host_ts = format_overlay_timestamp(target_msg.getTimestamp())
                target_device_ts = format_overlay_timestamp(target_msg.getTimestampDevice())
                target_dt_ms = (to_ns(target_msg.getTimestampDevice()) - target_points_frame.timestamp_device_ns) / 1_000_000.0
                cv2.putText(
                    target_frame,
                    (
                        f"{target_replay_input.stream_id} seq={target_msg.getSequenceNum()} idx={frame_idx} "
                        f"ref_seq={reference_points_frame.sequence_num} gt_seq={target_points_frame.sequence_num}"
                    ),
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    target_frame,
                    f"host_ts= {target_host_ts} dev_ts= {target_device_ts}",
                    (12, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    target_frame,
                    f"ref_dt={target_dt_ms:+.1f}ms valid_ref={valid_ref_for_projection} gt={len(target_gt_points)} projected={len(projected_points)}",
                    (12, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 200, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    target_frame,
                    "GT: green  projected: blue",
                    (12, 112),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if frame_id_errors_px:
                    frame_mean_err = float(np.mean(frame_id_errors_px))
                    frame_median_err = float(np.median(frame_id_errors_px))
                    frame_error_label = (
                        f"id-match n={frame_id_match_count} "
                        f"mean={frame_mean_err:.2f}px median={frame_median_err:.2f}px"
                    )
                else:
                    frame_error_label = (
                        "id-match n=0 "
                        f"(projected_ids={projected_points_with_ids}, gt_ids={gt_points_with_ids})"
                    )
                cv2.putText(
                    target_frame,
                    frame_error_label,
                    (12, 140),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                add_frame_title(reference_frame, "REFERENCE")
                add_frame_title(target_frame, "TARGET")
                histogram_tile_width = max(reference_frame.shape[1], target_frame.shape[1])
                per_frame_histogram = render_error_histogram(
                    frame_id_errors_px,
                    title="Per-frame ID error histogram",
                    width=histogram_tile_width,
                )
                running_histogram = render_error_histogram(
                    all_id_errors_px,
                    title="Running-total ID error histogram",
                    width=histogram_tile_width,
                )
                combined_frame = concatenate_frames_grid_2x2(
                    top_left=reference_frame,
                    top_right=target_frame,
                    bottom_left=per_frame_histogram,
                    bottom_right=running_histogram,
                )
                if comparison_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    comparison_writer = cv2.VideoWriter(
                        str(comparison_video_path),
                        fourcc,
                        comparison_video_fps,
                        (combined_frame.shape[1], combined_frame.shape[0]),
                    )
                    if not comparison_writer.isOpened():
                        raise RuntimeError(f"Failed to open video writer for {comparison_video_path}")
                    print(f"Saving combined comparison video to: {comparison_video_path}")
                comparison_writer.write(combined_frame)
                cv2.imshow("aruco comparison", combined_frame)

                if not wait_for_step_key():
                    return

            print("Completed all frames from points file.")
        finally:
            if comparison_writer is not None:
                comparison_writer.release()
                print(f"Saved combined comparison video: {comparison_video_path}")
            if all_id_errors_px:
                total_mean_err = float(np.mean(all_id_errors_px))
                total_median_err = float(np.median(all_id_errors_px))
                print(
                    "Final ID error summary: "
                    f"n={len(all_id_errors_px)} mean={total_mean_err:.2f}px median={total_median_err:.2f}px"
                )
            else:
                print("Final ID error summary: n=0 (no matched ID errors)")
            if all_roundtrip_errors_px:
                roundtrip_values = np.asarray(all_roundtrip_errors_px, dtype=np.float64)
                print(
                    "Final reference self-roundtrip summary: "
                    f"n={len(all_roundtrip_errors_px)} "
                    f"mean={float(np.mean(roundtrip_values)):.3f}px "
                    f"median={float(np.median(roundtrip_values)):.3f}px "
                    f"p95={float(np.percentile(roundtrip_values, 95.0)):.3f}px "
                    f"max={float(np.max(roundtrip_values)):.3f}px"
                )
            else:
                print("Final reference self-roundtrip summary: n=0")


if __name__ == "__main__":
    main()
