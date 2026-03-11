#!/usr/bin/env python3

import argparse
import json
import math
import os
from bisect import bisect_left
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import depthai as dai
import numpy as np

try:
    from mcap.reader import make_reader
except ModuleNotFoundError as exc:
    raise RuntimeError("mcap package is required. Install it in the same environment used for this script.") from exc

VIDEO_EXTENSIONS = (".mp4", ".avi", ".mkv")


@dataclass(frozen=True)
class PointsFrame:
    timestamp_us: int
    points: list[dict[str, Any]]


@dataclass(frozen=True)
class PointsCapture:
    path: Path
    socket: str
    rectified: bool
    frames: list[PointsFrame]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read capture folder produced by new_save_data_points.py and visualize 3D remapped "
            "reference points against target ground-truth points."
        )
    )
    parser.add_argument(
        "capture_folder",
        nargs="?",
        default="capture/default",
        help="Folder with reference/target videos and *_APRIL_POINTS.mcap files.",
    )
    parser.add_argument(
        "--step",
        action="store_true",
        help="Advance one frame at a time (press any key; q/ESC to quit).",
    )
    parser.add_argument(
        "--wait-ms",
        type=int,
        default=1,
        help="waitKey delay in milliseconds when not using --step.",
    )
    return parser.parse_args()


def to_us(ts: Any) -> int:
    if ts is None:
        return 0
    if isinstance(ts, datetime):
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        return int(ts.timestamp() * 1_000_000)
    if isinstance(ts, int):
        return ts
    if isinstance(ts, float):
        return int(ts * 1_000_000)
    if hasattr(ts, "total_seconds"):
        return int(ts.total_seconds() * 1_000_000)
    return 0


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


def load_points_frames(points_mcap_path: Path) -> list[PointsFrame]:
    frames: list[PointsFrame] = []
    with points_mcap_path.open("rb") as f:
        reader = make_reader(f)
        for _, _, message in reader.iter_messages():
            payload = json.loads(bytes(message.data).decode("utf-8"))
            timestamp_us = payload.get("timestamp_us")
            points = payload.get("points")
            if timestamp_us is None or points is None:
                continue
            frames.append(PointsFrame(timestamp_us=int(timestamp_us), points=list(points)))

    if not frames:
        raise RuntimeError(f"No points frames found in {points_mcap_path}")
    frames.sort(key=lambda frame: frame.timestamp_us)
    return frames


def resolve_points_captures(capture_folder: Path) -> tuple[PointsCapture, PointsCapture]:
    points_mcap_paths = sorted(capture_folder.glob("*APRIL_POINTS.mcap"))
    if not points_mcap_paths:
        raise FileNotFoundError(f"No *APRIL_POINTS.mcap files found in {capture_folder}")

    captures_by_type: dict[str, PointsCapture] = {}
    for path in points_mcap_paths:
        config = load_camera_config(path)
        point_type = str(config.get("point_type", "")).strip().lower()
        if point_type not in {"reference", "target"}:
            continue
        if point_type in captures_by_type:
            raise RuntimeError(
                f"Multiple {point_type} points files found: "
                f"{captures_by_type[point_type].path} and {path}"
            )
        captures_by_type[point_type] = PointsCapture(
            path=path,
            socket=str(config.get("socket", "")).strip(),
            rectified=bool(config.get("rectified", False)),
            frames=load_points_frames(path),
        )

    reference_capture = captures_by_type.get("reference")
    target_capture = captures_by_type.get("target")
    if reference_capture is None or target_capture is None:
        raise RuntimeError(
            "Could not resolve both reference and target points files from camera_config metadata."
        )
    if not reference_capture.socket:
        raise RuntimeError(f"Reference points metadata is missing socket in {reference_capture.path}")
    if not target_capture.socket:
        raise RuntimeError(f"Target points metadata is missing socket in {target_capture.path}")

    return reference_capture, target_capture


def list_capture_videos(capture_folder: Path) -> list[Path]:
    return sorted(path for path in capture_folder.iterdir() if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS)


def find_replay_video_and_metadata(
    capture_folder: Path,
    socket: str,
    role: str,
    rectified: bool,
) -> tuple[Path, Path]:
    video_files = list_capture_videos(capture_folder)
    role_candidates = [path for path in video_files if role in path.stem]
    candidates = [path for path in role_candidates if socket in path.stem]

    if role == "target":
        rectified_tag = f"{socket}_R"
        if rectified:
            preferred = [path for path in candidates if rectified_tag in path.stem]
            if preferred:
                candidates = preferred
        else:
            preferred = [path for path in candidates if rectified_tag not in path.stem]
            if preferred:
                candidates = preferred

    if not candidates and len(role_candidates) == 1:
        candidates = role_candidates

    if len(candidates) != 1:
        raise RuntimeError(
            f"Could not resolve unique {role} video for socket {socket}. Candidates: "
            f"{[str(path) for path in candidates] if candidates else [str(path) for path in role_candidates]}"
        )

    video_path = candidates[0]
    metadata_path = video_path.with_name(f"{video_path.stem}_metadata.mcap")
    if not metadata_path.exists():
        raise FileNotFoundError(f"Replay metadata file missing for {video_path}: {metadata_path}")

    return video_path, metadata_path


def create_replay_queue(pipeline: dai.Pipeline, video_path: Path, metadata_path: Path) -> Any:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(video_path)
    replay.setReplayMetadataFile(metadata_path)
    replay.setLoop(False)
    return replay.out.createOutputQueue(maxSize=1, blocking=True)


def get_nearest_imgframe_for_ts_us(
    queue: Any,
    target_ts_us: int,
    pending: dai.ImgFrame | None,
) -> tuple[dai.ImgFrame | None, dai.ImgFrame | None]:
    prev = pending
    while True:
        try:
            candidate = queue.get()
        except dai.MessageQueue.QueueException:
            return prev, None

        if not isinstance(candidate, dai.ImgFrame):
            continue

        candidate_ts_us = to_us(candidate.getTimestampDevice())
        if candidate_ts_us <= 0:
            candidate_ts_us = to_us(candidate.getTimestamp())
        if candidate_ts_us >= target_ts_us:
            if prev is None:
                return candidate, None
            prev_ts_us = to_us(prev.getTimestampDevice())
            if prev_ts_us <= 0:
                prev_ts_us = to_us(prev.getTimestamp())
            if abs(prev_ts_us - target_ts_us) <= abs(candidate_ts_us - target_ts_us):
                return prev, candidate
            return candidate, None

        prev = candidate


def nearest_points_frame(
    points_frames: list[PointsFrame], timestamps: list[int], target_ts_us: int
) -> PointsFrame:
    idx = bisect_left(timestamps, target_ts_us)

    if idx <= 0:
        return points_frames[0]
    if idx >= len(points_frames):
        return points_frames[-1]

    before = points_frames[idx - 1]
    after = points_frames[idx]
    if abs(after.timestamp_us - target_ts_us) < abs(target_ts_us - before.timestamp_us):
        return after
    return before


def to_pixel(point: dict[str, Any], width: int, height: int) -> tuple[float, float] | None:
    x = point.get("pixel_x")
    y = point.get("pixel_y")
    if x is None or y is None:
        return None

    x = float(x)
    y = float(y)
    if 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0:
        return (x * float(width), y * float(height))
    return (x, y)


def remap_reference_points_to_target(
    reference_points: list[dict[str, Any]],
    reference_transform: dai.ImgTransformation,
    target_transform: dai.ImgTransformation,
    reference_width: int,
    reference_height: int,
) -> list[tuple[float, float]]:
    intrinsics = np.asarray(reference_transform.getIntrinsicMatrix(), dtype=np.float64)
    fx = float(intrinsics[0, 0])
    fy = float(intrinsics[1, 1])
    cx = float(intrinsics[0, 2])
    cy = float(intrinsics[1, 2])

    projected_points: list[tuple[float, float]] = []
    for point in reference_points:
        depth_mm = point.get("depth_mm")
        if depth_mm is None:
            continue
        depth_mm = float(depth_mm)
        if not math.isfinite(depth_mm) or depth_mm <= 0:
            continue

        pixel = to_pixel(point, reference_width, reference_height)
        if pixel is None:
            continue
        x_px, y_px = pixel

        z_cm = depth_mm / 10.0
        x_cm = ((x_px - cx) * depth_mm / fx) / 10.0
        y_cm = ((y_px - cy) * depth_mm / fy) / 10.0

        source_3d = dai.Point3f(float(x_cm), float(y_cm), float(z_cm))
        projected_2d = reference_transform.project3DPointTo(target_transform, source_3d)

        if not (math.isfinite(projected_2d.x) and math.isfinite(projected_2d.y)):
            continue
        projected_points.append((float(projected_2d.x), float(projected_2d.y)))

    return projected_points


def extract_target_gt_points(
    target_points: list[dict[str, Any]], width: int, height: int
) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for point in target_points:
        pixel = to_pixel(point, width, height)
        if pixel is None:
            continue
        x, y = pixel
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        out.append((x, y))
    return out


def calculate_closest_rmse(
    predicted_points: list[tuple[float, float]],
    gt_points: list[tuple[float, float]],
) -> tuple[float | None, int, int]:
    if not predicted_points or not gt_points:
        return None, 0, 0

    pred_array = np.asarray(predicted_points, dtype=np.float64)
    nearest_distances: list[float] = []
    for gx, gy in gt_points:
        diffs = pred_array - np.array([gx, gy], dtype=np.float64)
        dists = np.linalg.norm(diffs, axis=1)
        nearest_distance = float(np.min(dists))
        if math.isfinite(nearest_distance):
            nearest_distances.append(nearest_distance)

    if not nearest_distances:
        return None, 0, 0

    keep_count = min(len(gt_points), len(predicted_points))
    dropped = max(0, len(nearest_distances) - keep_count)
    if dropped > 0:
        nearest_distances.sort(reverse=True)
        nearest_distances = nearest_distances[dropped:]

    mse = float(np.mean(np.square(np.asarray(nearest_distances, dtype=np.float64))))
    rmse = float(math.sqrt(mse))
    return rmse, len(nearest_distances), dropped


def draw_points(frame: np.ndarray, points: list[tuple[float, float]], color: tuple[int, int, int]) -> None:
    for x, y in points:
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        cv2.circle(frame, (int(round(x)), int(round(y))), 4, color, -1, cv2.LINE_AA)


def ensure_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim == 3 and frame.shape[2] == 1:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def wait_key(step_mode: bool, wait_ms: int) -> int:
    delay = 0 if step_mode else max(1, wait_ms)
    return cv2.waitKey(delay) & 0xFF


def main() -> None:

    args = parse_args()
    capture_folder = Path(args.capture_folder).expanduser().resolve()
    if not capture_folder.exists():
        raise FileNotFoundError(f"Capture folder does not exist: {capture_folder}")

    reference_capture, target_capture = resolve_points_captures(capture_folder)
    reference_video, reference_metadata = find_replay_video_and_metadata(
        capture_folder, reference_capture.socket, role="reference", rectified=reference_capture.rectified
    )
    target_video, target_metadata = find_replay_video_and_metadata(
        capture_folder, target_capture.socket, role="target", rectified=target_capture.rectified
    )

    print(f"Capture folder: {capture_folder}")
    print(f"Reference points: {reference_capture.path}")
    print(f"Target points (GT): {target_capture.path}")
    print(f"Reference replay: {reference_video}")
    print(f"Target replay: {target_video}")
    print("Controls: press q/ESC to quit.")
    if args.step:
        print("Step mode: press any key for next frame.")

    reference_window_name = f"reference_frame_{reference_capture.socket}"
    target_window_name = f"target_frame_{target_capture.socket}"

    with dai.Pipeline() as pipeline:
        reference_queue = create_replay_queue(pipeline, reference_video, reference_metadata)
        target_queue = create_replay_queue(pipeline, target_video, target_metadata)
        pipeline.start()

        pending_reference: dai.ImgFrame | None = None
        pending_target: dai.ImgFrame | None = None
        reference_ts = [frame.timestamp_us for frame in reference_capture.frames]

        for frame_idx, target_points_frame in enumerate(target_capture.frames):
            target_ts_us = target_points_frame.timestamp_us
            reference_points_frame = nearest_points_frame(
                reference_capture.frames, reference_ts, target_ts_us
            )

            reference_img, pending_reference = get_nearest_imgframe_for_ts_us(
                reference_queue, reference_points_frame.timestamp_us, pending_reference
            )
            target_img, pending_target = get_nearest_imgframe_for_ts_us(
                target_queue, target_ts_us, pending_target
            )

            if reference_img is None or target_img is None:
                print(f"Replay ended at frame {frame_idx}.")
                break

            reference_transform = reference_img.getTransformation()
            target_transform = target_img.getTransformation()

            predicted_points = remap_reference_points_to_target(
                reference_points=reference_points_frame.points,
                reference_transform=reference_transform,
                target_transform=target_transform,
                reference_width=int(reference_img.getWidth()),
                reference_height=int(reference_img.getHeight()),
            )
            gt_points = extract_target_gt_points(
                target_points_frame.points,
                width=int(target_img.getWidth()),
                height=int(target_img.getHeight()),
            )

            rmse_px, used_pairs, dropped_gt = calculate_closest_rmse(predicted_points, gt_points)

            reference_frame = ensure_bgr(reference_img.getCvFrame().copy())
            frame = ensure_bgr(target_img.getCvFrame().copy())
            draw_points(frame, gt_points, (0, 255, 0))
            draw_points(frame, predicted_points, (255, 0, 0))

            dt_ms = (reference_points_frame.timestamp_us - target_ts_us) / 1000.0
            cv2.putText(
                frame,
                f"idx={frame_idx} gt={len(gt_points)} pred={len(predicted_points)} used={used_pairs}",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"rmse_px={rmse_px:.3f}" if rmse_px is not None else "rmse_px=n/a",
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"ref-gt dt={dt_ms:+.1f}ms dropped_gt={dropped_gt}",
                (12, 86),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                "GT: green  Predicted(remap3DPointTo): blue",
                (12, 114),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            print(
                f"frame={frame_idx} gt={len(gt_points)} pred={len(predicted_points)} "
                f"used={used_pairs} dropped_gt={dropped_gt} "
                f"rmse_px={(f'{rmse_px:.6f}' if rmse_px is not None else 'n/a')}"
            )
            cv2.imshow(reference_window_name, reference_frame)
            cv2.imshow(target_window_name, frame)

            key = wait_key(args.step, args.wait_ms)
            if key in (ord("q"), 27):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
