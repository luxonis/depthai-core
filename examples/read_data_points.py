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
    from mcap.exceptions import InvalidMagic
    from mcap.records import Channel, Message, Schema
    from mcap.stream_reader import StreamReader
except ModuleNotFoundError as exc:
    raise RuntimeError("mcap package is required. Install it in the same environment used for this script.") from exc


CAMERA_PATTERN = re.compile(r"(CAM_[A-Z](?:_R)?)$")
CAMERA_ORDER = ["CAM_A", "CAM_B", "CAM_C", "CAM_D", "CAM_E", "CAM_B_R", "CAM_C_R"]


@dataclass(frozen=True)
class CameraReplayInput:
    camera: str
    video: Path
    metadata: Path | None


@dataclass(frozen=True)
class PointsFrame:
    reference_camera: str
    april_sequence_num: int
    rgb_sequence_num: int
    timestamp_device_ns: int
    mode: str
    points: list[dict[str, Any]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read capture folder produced by save_data_points.py, replay all camera videos, "
            "and visualize reprojections from the reference camera to every other camera."
        )
    )
    parser.add_argument(
        "capture_folder",
        nargs="?",
        default="capture/default",
        help="Folder with *_CAM_*.mp4, *_metadata.mcap and *__APRIL_POINTS.mcap files.",
    )
    parser.add_argument(
        "--points-mcap",
        type=Path,
        default=None,
        help="Optional explicit path to *__APRIL_POINTS.mcap.",
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
    return match.group(1)


def find_camera_inputs(capture_folder: Path) -> list[CameraReplayInput]:
    found: dict[str, CameraReplayInput] = {}
    for video_path in sorted(capture_folder.glob("*_CAM_*.mp4")):
        camera = detect_camera_name(video_path)
        if camera is None:
            continue

        metadata_path = video_path.with_name(f"{video_path.stem}_metadata.mcap")
        metadata = metadata_path if metadata_path.exists() else None
        found[camera] = CameraReplayInput(camera=camera, video=video_path, metadata=metadata)

    if not found:
        raise FileNotFoundError(f"No camera videos found in {capture_folder}")

    cameras = list(found.values())
    cameras.sort(key=lambda item: camera_sort_key(item.camera))
    return cameras


def resolve_points_mcap(capture_folder: Path, explicit_path: Path | None) -> Path:
    if explicit_path is not None:
        path = explicit_path if explicit_path.is_absolute() else capture_folder / explicit_path
        if not path.exists():
            raise FileNotFoundError(f"Points MCAP does not exist: {path}")
        return path

    matches = sorted(capture_folder.glob("*__APRIL_POINTS.mcap"))
    if len(matches) == 0:
        raise FileNotFoundError(f"No *__APRIL_POINTS.mcap file found in {capture_folder}")
    if len(matches) > 1:
        raise RuntimeError(
            "Multiple points MCAP files found. Pass one explicitly via --points-mcap.\n"
            + "\n".join(str(path) for path in matches)
        )
    return matches[0]


def resolve_depth_mcap(capture_folder: Path, explicit_path: Path | None) -> Path | None:
    if explicit_path is not None:
        path = explicit_path if explicit_path.is_absolute() else capture_folder / explicit_path
        if not path.exists():
            raise FileNotFoundError(f"Depth MCAP does not exist: {path}")
        return path

    aligned_matches = sorted(capture_folder.glob("*_DEPTH_ALIGNED.mcap"))
    if len(aligned_matches) == 1:
        return aligned_matches[0]
    if len(aligned_matches) > 1:
        raise RuntimeError(
            "Multiple *_DEPTH_ALIGNED.mcap files found. Pass one explicitly via --depth-mcap.\n"
            + "\n".join(str(path) for path in aligned_matches)
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


def load_points_frames(points_mcap_path: Path) -> list[PointsFrame]:
    frames: list[PointsFrame] = []
    try:
        with points_mcap_path.open("rb") as f:
            reader = make_reader(f)
            for _, channel, message in reader.iter_messages():
                if channel.topic not in {"apriltag/points", "apriltag/points_spatial"}:
                    continue
                payload = json.loads(bytes(message.data).decode("utf-8"))
                if "points_spatial_mm" in payload:
                    mode = "spatial_mm"
                    points = payload["points_spatial_mm"]
                elif "points_normalized" in payload:
                    mode = "normalized"
                    points = payload["points_normalized"]
                else:
                    raise RuntimeError("Unsupported points payload: expected points_spatial_mm or points_normalized")
                frames.append(
                    PointsFrame(
                        reference_camera=str(payload["reference_camera"]),
                        april_sequence_num=int(payload["april_sequence_num"]),
                        rgb_sequence_num=int(payload["rgb_sequence_num"]),
                        timestamp_device_ns=int(payload.get("timestamp_device_ns", 0)),
                        mode=mode,
                        points=points,
                    )
                )
    except InvalidMagic:
        # Backward compatibility for points files created without Writer.start()
        # (record stream + trailing magic/footer, but missing initial MCAP magic/header).
        print(f"Warning: {points_mcap_path} is missing MCAP header; reading in compatibility mode.")
        channel_by_id: dict[int, Channel] = {}
        with points_mcap_path.open("rb") as f:
            for record in StreamReader(f, skip_magic=True, emit_chunks=False).records:
                if isinstance(record, Schema):
                    continue
                if isinstance(record, Channel):
                    channel_by_id[int(record.id)] = record
                    continue
                if not isinstance(record, Message):
                    continue

                channel = channel_by_id.get(int(record.channel_id))
                if channel is None:
                    continue
                if channel.topic not in {"apriltag/points", "apriltag/points_spatial"}:
                    continue

                payload = json.loads(bytes(record.data).decode("utf-8"))
                if "points_spatial_mm" in payload:
                    mode = "spatial_mm"
                    points = payload["points_spatial_mm"]
                elif "points_normalized" in payload:
                    mode = "normalized"
                    points = payload["points_normalized"]
                else:
                    raise RuntimeError("Unsupported points payload: expected points_spatial_mm or points_normalized")
                frames.append(
                    PointsFrame(
                        reference_camera=str(payload["reference_camera"]),
                        april_sequence_num=int(payload["april_sequence_num"]),
                        rgb_sequence_num=int(payload["rgb_sequence_num"]),
                        timestamp_device_ns=int(payload.get("timestamp_device_ns", 0)),
                        mode=mode,
                        points=points,
                    )
                )

    if not frames:
        raise RuntimeError(f"No points frames found in {points_mcap_path}")
    return frames


def to_reprojected_from_normalized(
    reference_transform: dai.ImgTransformation,
    target_transform: dai.ImgTransformation,
    normalized_points: list[dict[str, Any]],
    ref_width: int,
    ref_height: int,
) -> list[tuple[float, float, bool]]:
    reprojected: list[tuple[float, float, bool]] = []
    for point in normalized_points:
        source = dai.Point2f(float(point["x"]) * ref_width, float(point["y"]) * ref_height)
        projected = reference_transform.remapPointTo(target_transform, source)
        reprojected.append((float(projected.x), float(projected.y), True))
    return reprojected


def to_reprojected_from_spatial_mm(
    reference_transform: dai.ImgTransformation,
    target_transform: dai.ImgTransformation,
    spatial_points: list[dict[str, Any]],
) -> list[tuple[float, float, bool]]:
    reprojected: list[tuple[float, float, bool]] = []
    for point in spatial_points:
        x = point.get("x")
        y = point.get("y")
        z = point.get("z")
        point_has_xyz = x is not None and y is not None and z is not None
        valid_field = point.get("valid")
        # Backward-compatibility: old captures may omit "valid" but still provide x/y/z.
        point_valid = bool(valid_field) if valid_field is not None else point_has_xyz

        if point_valid and point_has_xyz:
            source_3d_cm = dai.Point3f(float(x) / 10.0, float(y) / 10.0, float(z) / 10.0)
            projected = reference_transform.project3DPointTo(target_transform, source_3d_cm)
            reprojected.append((float(projected.x), float(projected.y), True))
            continue

        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue
        fallback_2d = reference_transform.remapPointTo(target_transform, dai.Point2f(float(px), float(py)))
        reprojected.append((float(fallback_2d.x), float(fallback_2d.y), False))

    return reprojected


def sample_depth_mm(depth_frame: np.ndarray, x: int, y: int, radius: int = 1) -> int | None:
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


def to_reprojected_from_depth_aligned(
    depth_transform: dai.ImgTransformation,
    target_transform: dai.ImgTransformation,
    spatial_points: list[dict[str, Any]],
    depth_frame_mm: np.ndarray,
) -> list[tuple[float, float, bool]]:
    intr = np.asarray(depth_transform.getIntrinsicMatrix(), dtype=np.float64)
    fx = float(intr[0, 0])
    fy = float(intr[1, 1])
    cx = float(intr[0, 2])
    cy = float(intr[1, 2])

    reprojected: list[tuple[float, float, bool]] = []
    for point in spatial_points:
        px = point.get("pixel_x")
        py = point.get("pixel_y")
        if px is None or py is None:
            continue

        x_px = int(round(float(px)))
        y_px = int(round(float(py)))
        depth_mm = sample_depth_mm(depth_frame_mm, x_px, y_px, radius=1)
        if depth_mm is None:
            fallback_2d = depth_transform.remapPointTo(target_transform, dai.Point2f(float(x_px), float(y_px)))
            reprojected.append((float(fallback_2d.x), float(fallback_2d.y), False))
            continue

        z_cm = float(depth_mm) / 10.0
        x_cm = ((float(x_px) - cx) * float(depth_mm) / fx) / 10.0
        y_cm = ((float(y_px) - cy) * float(depth_mm) / fy) / 10.0
        source_3d_cm = dai.Point3f(x_cm, y_cm, z_cm)

        projected = depth_transform.project3DPointTo(target_transform, source_3d_cm)
        reprojected.append((float(projected.x), float(projected.y), True))

    return reprojected


def draw_points(frame: Any, points: list[tuple[float, float, bool]]) -> None:
    for x, y, valid in points:
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.circle(frame, (int(round(x)), int(round(y))), 4, color, -1, cv2.LINE_AA)


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
        queues[cam.camera] = replay.out.createOutputQueue(maxSize=1, blocking=True)
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


def overlay_depth_on_frame(frame_bgr: np.ndarray, depth_mm: np.ndarray, alpha: float = 0.45) -> np.ndarray:
    frame_h, frame_w = frame_bgr.shape[:2]
    if depth_mm.shape[:2] != (frame_h, frame_w):
        depth_mm = cv2.resize(depth_mm, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)

    depth_color = depth_to_colormap(depth_mm)
    blended = cv2.addWeighted(frame_bgr, 1.0 - alpha, depth_color, alpha, 0.0)

    valid_mask = depth_mm > 0
    out = frame_bgr.copy()
    out[valid_mask] = blended[valid_mask]
    return out


def main() -> None:
    if os.environ.get("DISPLAY", "") == "":
        raise RuntimeError("DISPLAY is not set. This script uses cv2.imshow and must run in a GUI session.")

    args = parse_args()
    capture_folder = Path(args.capture_folder).expanduser().resolve()
    if not capture_folder.exists():
        raise FileNotFoundError(f"Capture folder does not exist: {capture_folder}")

    camera_inputs = find_camera_inputs(capture_folder)
    points_mcap = resolve_points_mcap(capture_folder, args.points_mcap)
    depth_mcap = resolve_depth_mcap(capture_folder, args.depth_mcap)
    points_frames = load_points_frames(points_mcap)

    available_cameras = {cam.camera for cam in camera_inputs}
    reference_camera = points_frames[0].reference_camera
    if reference_camera not in available_cameras:
        raise RuntimeError(
            f"Reference camera {reference_camera} from points file is missing. "
            f"Found cameras: {sorted(available_cameras)}"
        )

    print(f"Capture folder: {capture_folder}")
    print(f"Points file: {points_mcap}")
    if depth_mcap is None:
        print("Depth file: not found (depth visualization disabled)")
    else:
        print(f"Depth file: {depth_mcap}")
    if points_frames and points_frames[0].mode == "spatial_mm":
        usable = 0
        for p in points_frames[0].points:
            has_xyz = p.get("x") is not None and p.get("y") is not None and p.get("z") is not None
            valid_field = p.get("valid")
            point_valid = bool(valid_field) if valid_field is not None else has_xyz
            if point_valid:
                usable += 1
        total = len(points_frames[0].points)
        print(f"Spatial points usable for 3D reprojection in first frame: {usable}/{total}")
    print(f"Reference camera: {reference_camera}")
    print(f"Cameras: {[cam.camera for cam in camera_inputs]}")
    print("Controls: press 'l' for next frame, 'q' or ESC to quit.")

    with dai.Pipeline() as pipeline:
        queues = create_replay_queues(pipeline, camera_inputs)
        depth_queue = create_depth_replay_queue(pipeline, depth_mcap) if depth_mcap is not None else None
        pipeline.start()
        printed_extrinsics = False
        printed_spatial_source = False
        pending_depth: dai.ImgFrame | None = None
        pending_by_camera: dict[str, dai.ImgFrame | None] = {cam.camera: None for cam in camera_inputs}

        for frame_idx, points_frame in enumerate(points_frames):
            assert points_frame.reference_camera == reference_camera, (
                f"Reference camera changed in points file at frame {frame_idx}: "
                f"{points_frame.reference_camera} != {reference_camera}"
            )
            frames: dict[str, dai.ImgFrame] = {}
            if points_frame.timestamp_device_ns > 0:
                for cam in camera_inputs:
                    img, pending_by_camera[cam.camera] = get_nearest_imgframe_for_reference_ts(
                        queues[cam.camera],
                        points_frame.timestamp_device_ns,
                        pending_by_camera[cam.camera],
                    )
                    if img is None:
                        print(f"Replay ended at frame {frame_idx}.")
                        return
                    frames[cam.camera] = img
            else:
                for cam in camera_inputs:
                    try:
                        img = queues[cam.camera].get()
                    except dai.MessageQueue.QueueException:
                        print(f"Replay ended at frame {frame_idx}.")
                        return
                    assert isinstance(img, dai.ImgFrame)
                    frames[cam.camera] = img

            # seq_by_camera = {cam: int(msg.getSequenceNum()) for cam, msg in frames.items()}
            # first_seq = next(iter(seq_by_camera.values()))
            # for cam, seq in seq_by_camera.items():
            #     assert seq == first_seq, f"Sequence mismatch at frame {frame_idx}: {seq_by_camera}"

            # assert first_seq == points_frame.rgb_sequence_num, (
            #     f"Points file mismatch at frame {frame_idx}: replay seq={first_seq}, "
            #     f"points rgb_sequence_num={points_frame.rgb_sequence_num}"
            # )

            ref_frame = frames[reference_camera]
            ref_transform = ref_frame.getTransformation()
            ref_w = int(ref_frame.getWidth())
            ref_h = int(ref_frame.getHeight())
            ref_device_ts_ns = to_ns(ref_frame.getTimestampDevice())

            depth_msg: dai.ImgFrame | None = None
            depth_dt_ms: float | None = None
            depth_frame_mm: np.ndarray | None = None
            depth_transform: dai.ImgTransformation | None = None
            if depth_queue is not None:
                depth_msg, pending_depth = get_nearest_imgframe_for_reference_ts(depth_queue, ref_device_ts_ns, pending_depth)
                if depth_msg is not None:
                    depth_dt_ms = (to_ns(depth_msg.getTimestampDevice()) - ref_device_ts_ns) / 1_000_000.0
                    depth_frame_mm = depth_msg.getFrame()
                    depth_transform = depth_msg.getTransformation()

            if not printed_extrinsics:
                print("Reference->target extrinsics translation (cm):")
                for cam in camera_inputs:
                    target_transform = frames[cam.camera].getTransformation()
                    try:
                        matrix = ref_transform.getExtrinsicsTransformationMatrixTo(target_transform)
                        tx = float(matrix[0][3])
                        ty = float(matrix[1][3])
                        tz = float(matrix[2][3])
                        print(f"  {reference_camera} -> {cam.camera}: [{tx:.3f}, {ty:.3f}, {tz:.3f}]")
                    except Exception as exc:
                        print(f"  {reference_camera} -> {cam.camera}: unavailable ({exc})")
                printed_extrinsics = True

            for cam in camera_inputs:
                msg = frames[cam.camera]
                frame = msg.getCvFrame()
                if depth_msg is not None and cam.camera == reference_camera:
                    frame = overlay_depth_on_frame(frame, depth_msg.getFrame(), alpha=0.45)
                target_transform = msg.getTransformation()

                if points_frame.mode == "spatial_mm":
                    if depth_frame_mm is not None and depth_transform is not None:
                        if not printed_spatial_source:
                            print("Spatial reprojection source: aligned depth replay (pixel_x/pixel_y + depth frame)")
                            printed_spatial_source = True
                        projected = to_reprojected_from_depth_aligned(depth_transform, target_transform, points_frame.points, depth_frame_mm)
                    else:
                        projected = to_reprojected_from_spatial_mm(ref_transform, target_transform, points_frame.points)
                else:
                    projected = to_reprojected_from_normalized(ref_transform, target_transform, points_frame.points, ref_w, ref_h)

                draw_points(frame, projected)
                host_ts = format_overlay_timestamp(msg.getTimestamp())
                device_ts = format_overlay_timestamp(msg.getTimestampDevice())
                cv2.putText(
                    frame,
                    f"{cam.camera} seq={msg.getSequenceNum()} idx={frame_idx}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame,
                    f"host_ts= {host_ts} dev_ts= {device_ts}",
                    (12, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                if points_frame.timestamp_device_ns > 0:
                    cam_dt_ms = (to_ns(msg.getTimestampDevice()) - points_frame.timestamp_device_ns) / 1_000_000.0
                    cv2.putText(
                        frame,
                        f"points_dt={cam_dt_ms:+.1f}ms",
                        (12, 84),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 200, 255),
                        2,
                        cv2.LINE_AA,
                    )
                if depth_msg is not None and cam.camera == reference_camera:
                    cv2.putText(
                        frame,
                        f"depth seq={depth_msg.getSequenceNum()} dt={depth_dt_ms:+.1f}ms",
                        (12, 112),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                cv2.imshow(cam.camera, frame)

            if not wait_for_step_key():
                return

        print("Completed all frames from points file.")


if __name__ == "__main__":
    main()
