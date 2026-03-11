import argparse
import json
import signal
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import depthai as dai
import numpy as np
from mcap.writer import Writer


ARUCO_DICT: Any = None
ARUCO_PARAMS: Any = None
ARUCO_BOARD: Any = None
ARUCO_DETECTOR: Any = None
ARUCO_REFINE = True


def ensure_bgr(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim == 3 and frame.shape[2] == 1:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def sample_depth_mm(depth_frame: np.ndarray | None, x: int, y: int, radius: int = 1) -> Optional[int]:
    if depth_frame is None:
        return None

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


def to_us(ts: Any) -> int:
    if ts is None:
        return 0
    if isinstance(ts, timedelta):
        return int(ts.total_seconds() * 1_000_000)
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
    raise TypeError(f"Unsupported timestamp type: {type(ts)}")


def link_record_video_node(
    pipeline: dai.Pipeline,
    output: dai.Node.Output,
    output_path: str,
    fps: Optional[int] = None,
) -> None:
    record_video = pipeline.create(dai.node.RecordVideo)
    record_video.setRecordVideoFile(Path(f"{output_path}.mp4"))
    record_video.setRecordMetadataFile(Path(f"{output_path}_metadata.mcap"))
    if fps is not None:
        record_video.setFps(int(fps))
    output.link(record_video.input)


def link_record_metadata_node(pipeline: dai.Pipeline, output: dai.Node.Output, output_path: str) -> None:
    record_metadata = pipeline.create(dai.node.RecordMetadataOnly)
    record_metadata.setRecordFile(Path(f"{output_path}.mcap"))
    output.link(record_metadata.input)


def create_mcap_writer(output_path: Path, topic: str, metadata: dict[str, Any]):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mcap_fp = output_path.open("wb")
    writer = Writer(mcap_fp)
    writer.start()

    writer.add_metadata(
        name="camera_config",
        data={"json": json.dumps(metadata, separators=(",", ":"))},
    )

    points_item_schema = {
        "type": "object",
        "properties": {
            "pixel_x": {"type": "number"},
            "pixel_y": {"type": "number"},
            "depth_mm": {"type": ["number", "null"]},
            "valid": {"type": "boolean"},
            "aruco_id": {"type": "integer"},
            "marker_corner_index": {"type": "integer"},
            "charuco_corner_id": {"type": "integer"},
        },
        "required": [
            "pixel_x",
            "pixel_y",
            "depth_mm",
            "valid",
            "aruco_id",
            "marker_corner_index",
            "charuco_corner_id",
        ],
    }

    schema = {
        "type": "object",
        "properties": {
            "timestamp_us": {"type": "integer"},
            "points": {"type": "array", "items": points_item_schema},
        },
        "required": ["timestamp_us", "points"],
    }

    schema_id = writer.register_schema(
        name="depthai.aruco.points",
        encoding="jsonschema",
        data=json.dumps(schema, separators=(",", ":")).encode("utf-8"),
    )
    channel_id = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema_id)
    return mcap_fp, writer, channel_id


def write_aruco_points(
    writer: Writer,
    channel_id: int,
    ts: int,
    data_points: Iterable[dict[str, Any]],
    seq_num: int,
) -> None:
    payload = {
        "timestamp_us": ts,
        "points": list(data_points),
    }

    writer.add_message(
        channel_id=channel_id,
        log_time=ts * 1000,
        publish_time=ts * 1000,
        sequence=seq_num,
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
    )


def create_writer_metadata(
    point_type: str,
    socket: str,
    undistortion_enabled: bool,
    size: tuple[int, int],
    resize_mode: str,
    rectified: bool,
):
    return {
        "point_type": point_type,
        "socket": socket,
        "undistortion_enabled": undistortion_enabled,
        "size": {"width": size[0], "height": size[1]},
        "resize_mode": resize_mode,
        "rectified": rectified,
    }


def resolve_aruco_dictionary(dictionary_name: str):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV ArUco module is unavailable. Install opencv-contrib-python.")

    dict_id = getattr(cv2.aruco, dictionary_name, None)
    if dict_id is None:
        raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")

    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def create_aruco_parameters():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def create_aruco_detector(aruco_dict, aruco_params):
    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return None


def create_aruco_board(
    aruco_dict: Any,
    board_cols: int,
    board_rows: int,
    marker_length: float,
    marker_separation: float,
):
    if board_cols <= 0 or board_rows <= 0:
        return None

    if hasattr(cv2.aruco, "GridBoard_create"):
        return cv2.aruco.GridBoard_create(board_cols, board_rows, marker_length, marker_separation, aruco_dict)

    try:
        return cv2.aruco.GridBoard((board_cols, board_rows), marker_length, marker_separation, aruco_dict)
    except Exception:
        return None


def calculate_aruco(img: np.ndarray) -> list[dict[str, float | int]]:
    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    if ARUCO_DETECTOR is not None:
        corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    if ids is None or len(ids) == 0:
        return []

    refined_corners, refined_ids = corners, ids
    if ARUCO_REFINE and ARUCO_BOARD is not None and hasattr(cv2.aruco, "refineDetectedMarkers"):
        try:
            refine_result = cv2.aruco.refineDetectedMarkers(gray, ARUCO_BOARD, corners, ids, rejected)
            if len(refine_result) >= 2:
                candidate_corners, candidate_ids = refine_result[0], refine_result[1]
                if candidate_ids is not None and len(candidate_ids) > 0:
                    refined_corners, refined_ids = candidate_corners, candidate_ids
        except cv2.error:
            pass

    ordered_markers = sorted(
        zip(refined_ids.flatten().tolist(), refined_corners),
        key=lambda item: int(item[0]),
    )

    points: list[dict[str, float | int]] = []
    for marker_id, marker_corners in ordered_markers:
        for corner_index, (x, y) in enumerate(marker_corners.reshape(-1, 2)):
            # Stable per-corner identifier used to link points across views/frames.
            charuco_corner_id = int(marker_id) * 4 + int(corner_index)
            points.append(
                {
                    "x": float(x),
                    "y": float(y),
                    "aruco_id": int(marker_id),
                    "marker_corner_index": int(corner_index),
                    "charuco_corner_id": int(charuco_corner_id),
                }
            )
    return points


def aruco_points_to_mcap_points_list(
    aruco_points: Iterable[dict[str, float | int]],
    frame_width: int,
    frame_height: int,
    depth_frame: np.ndarray | None,
) -> list[dict[str, Optional[float]]]:
    width = max(1, int(frame_width))
    height = max(1, int(frame_height))

    points_spatial: list[dict[str, Optional[float]]] = []
    for point in aruco_points:
        x_px = int(round(float(point["x"])))
        y_px = int(round(float(point["y"])))
        aruco_id = int(point["aruco_id"])
        marker_corner_index = int(point["marker_corner_index"])
        charuco_corner_id = int(point["charuco_corner_id"])

        depth_mm = sample_depth_mm(depth_frame, x_px, y_px, radius=1)
        if depth_mm is None:
            points_spatial.append(
                {
                    "pixel_x": x_px / width,
                    "pixel_y": y_px / height,
                    "depth_mm": None,
                    "valid": False,
                    "aruco_id": aruco_id,
                    "marker_corner_index": marker_corner_index,
                    "charuco_corner_id": charuco_corner_id,
                }
            )
            continue

        points_spatial.append(
            {
                "pixel_x": x_px / width,
                "pixel_y": y_px / height,
                "depth_mm": float(depth_mm),
                "valid": True,
                "aruco_id": aruco_id,
                "marker_corner_index": marker_corner_index,
                "charuco_corner_id": charuco_corner_id,
            }
        )
    return points_spatial


def draw_aruco_points(
    frame: np.ndarray,
    aruco_points: list[dict[str, float | int]],
    spatial_points: Optional[list[dict[str, Optional[float]]]] = None,
) -> np.ndarray:
    vis = ensure_bgr(frame).copy()

    for marker_start in range(0, len(aruco_points), 4):
        marker_points = aruco_points[marker_start : marker_start + 4]
        if len(marker_points) < 4:
            break
        marker_pts = np.array(
            [[int(round(float(p["x"]))), int(round(float(p["y"])))] for p in marker_points],
            dtype=np.int32,
        )
        cv2.polylines(vis, [marker_pts], isClosed=True, color=(255, 180, 0), thickness=2)

    for idx, point in enumerate(aruco_points):
        x = float(point["x"])
        y = float(point["y"])
        color = (0, 255, 0)
        if spatial_points is not None and idx < len(spatial_points):
            color = (0, 255, 0) if spatial_points[idx].get("valid", False) else (0, 0, 255)
        cv2.circle(vis, (int(round(x)), int(round(y))), 4, color, -1)

    cv2.putText(
        vis,
        f"aruco_corners:{len(aruco_points)}",
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 180, 0),
        1,
        cv2.LINE_AA,
    )
    return vis


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output-path", default="capture/default/", help="Output path prefix")
    parser.add_argument("-d", "--duration", type=int, default=10, help="Duration in seconds")
    parser.add_argument("--fps", type=int, default=30, help="FPS for cameras and recording")
    parser.add_argument(
        "-rc",
        "--reference-camera",
        type=str,
        default="CAM_A",
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Reference camera used for ArUco corner detection.",
    )
    parser.add_argument(
        "-tc",
        "--target-camera",
        type=str,
        default="CAM_B",
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Target camera to save ArUco points from.",
    )
    parser.add_argument(
        "-r",
        "--rectified",
        "--rectification",
        dest="rectified",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use rectified target camera output. Only applicable if target camera is CAM_B or CAM_C.",
    )
    parser.add_argument(
        "-rcs",
        "--reference-camera-size",
        nargs=2,
        type=int,
        metavar=("W", "H"),
        default=(1280, 800),
        help="Resolution of the reference camera as W H",
    )
    parser.add_argument(
        "-tcs",
        "--target-camera-size",
        nargs=2,
        type=int,
        metavar=("W", "H"),
        default=(1280, 800),
        help="Resolution of the target camera as W H",
    )
    parser.add_argument(
        "-rcd",
        "--reference-camera-undistortion",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable undistortion for reference camera",
    )
    parser.add_argument(
        "-tcd",
        "--target-camera-undistortion",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable undistortion for target camera if different from reference camera",
    )
    parser.add_argument(
        "-rcrm",
        "--reference-camera-resize-mode",
        type=str,
        default="CROP",
        choices=["CROP", "STRETCH", "LETTERBOX"],
        help="Resize mode for reference camera",
    )
    parser.add_argument(
        "-tcrm",
        "--target-camera-resize-mode",
        type=str,
        default="CROP",
        choices=["CROP", "STRETCH", "LETTERBOX"],
        help="Resize mode for target camera",
    )
    parser.add_argument(
        "--aruco-dictionary",
        type=str,
        default="DICT_4X4_1000",
        help="OpenCV ArUco dictionary name (for example DICT_4X4_1000).",
    )
    parser.add_argument("--board-cols", type=int, default=5, help="ArUco grid board marker columns.")
    parser.add_argument("--board-rows", type=int, default=7, help="ArUco grid board marker rows.")
    parser.add_argument("--marker-length", type=float, default=0.04, help="ArUco marker length in meters.")
    parser.add_argument("--marker-separation", type=float, default=0.01, help="ArUco marker separation in meters.")
    parser.add_argument(
        "--refine-markers",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run board-aware refine step after marker detection.",
    )

    args = parser.parse_args()

    resize_modes = {
        "CROP": dai.ImgResizeMode.CROP,
        "STRETCH": dai.ImgResizeMode.STRETCH,
        "LETTERBOX": dai.ImgResizeMode.LETTERBOX,
    }

    output_path = args.output_path
    duration = args.duration
    rectified = args.rectified
    fps = args.fps
    reference_camera_name = args.reference_camera
    target_camera_name = args.target_camera
    ref_camera_size = tuple(args.reference_camera_size)
    target_camera_size = tuple(args.target_camera_size)
    ref_camera_undistortion = args.reference_camera_undistortion
    target_camera_undistortion = args.target_camera_undistortion
    ref_camera_resize_mode = resize_modes[args.reference_camera_resize_mode]
    target_camera_resize_mode = resize_modes[args.target_camera_resize_mode]

    global ARUCO_DICT, ARUCO_PARAMS, ARUCO_BOARD, ARUCO_DETECTOR, ARUCO_REFINE
    ARUCO_DICT = resolve_aruco_dictionary(args.aruco_dictionary)
    ARUCO_PARAMS = create_aruco_parameters()
    ARUCO_BOARD = create_aruco_board(
        ARUCO_DICT,
        board_cols=args.board_cols,
        board_rows=args.board_rows,
        marker_length=args.marker_length,
        marker_separation=args.marker_separation,
    )
    ARUCO_DETECTOR = create_aruco_detector(ARUCO_DICT, ARUCO_PARAMS)
    ARUCO_REFINE = args.refine_markers

    ref_metadata = create_writer_metadata(
        point_type="reference",
        socket=reference_camera_name,
        undistortion_enabled=ref_camera_undistortion,
        size=ref_camera_size,
        resize_mode=args.reference_camera_resize_mode,
        rectified=rectified,
    )
    ref_metadata.update(
        {
            "aruco_dictionary": args.aruco_dictionary,
            "board_cols": args.board_cols,
            "board_rows": args.board_rows,
        }
    )

    target_metadata = create_writer_metadata(
        point_type="target",
        socket=target_camera_name,
        undistortion_enabled=target_camera_undistortion,
        size=target_camera_size,
        resize_mode=args.target_camera_resize_mode,
        rectified=rectified,
    )
    target_metadata.update(
        {
            "aruco_dictionary": args.aruco_dictionary,
            "board_cols": args.board_cols,
            "board_rows": args.board_rows,
        }
    )

    ref_points_mcap_path = Path(f"{output_path}reference_ARUCO_POINTS.mcap")
    ref_mcap_file, ref_mcap_writer, ref_mcap_channel_id = create_mcap_writer(
        ref_points_mcap_path, topic="reference_points", metadata=ref_metadata
    )
    target_points_mcap_path = Path(f"{output_path}_target_ARUCO_POINTS.mcap")
    target_mcap_file, target_mcap_writer, target_mcap_channel_id = create_mcap_writer(
        target_points_mcap_path, topic="target_points", metadata=target_metadata
    )

    try:
        with dai.Pipeline() as pipeline:

            def signal_handler(sig, frame):
                print("Interrupted, stopping the pipeline")
                pipeline.stop()

            signal.signal(signal.SIGINT, signal_handler)

            sync_node = pipeline.create(dai.node.Sync)
            sync_node.setRunOnHost(False)
            sync_node.setSyncThreshold(timedelta(milliseconds=1000 / fps * 0.5))
            sync_node.setSyncAttempts(-1)

            demux = pipeline.create(dai.node.MessageDemux)
            sync_node.out.link(demux.input)

            camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            cameras = {"CAM_B": camera_b, "CAM_C": camera_c}

            reference_camera = cameras[reference_camera_name]
            reference_camera_output = reference_camera.requestOutput(
                ref_camera_size,
                fps=fps,
                enableUndistortion=ref_camera_undistortion,
                resizeMode=ref_camera_resize_mode,
            )
            reference_camera_output.link(sync_node.inputs["ref_camera"])
            link_record_video_node(
                pipeline,
                demux.outputs["ref_camera"],
                f"{output_path}{reference_camera_name}_reference",
                fps=fps,
            )

            camera_c_output = camera_c.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )
            camera_b_output = camera_b.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )

            stereo = pipeline.create(dai.node.StereoDepth)
            camera_b_output.link(stereo.left)
            camera_c_output.link(stereo.right)

            if reference_camera_name != "CAM_B" and not rectified:
                image_align = pipeline.create(dai.node.ImageAlign)
                stereo.depth.link(image_align.input)
                reference_camera_output.link(image_align.inputAlignTo)
                image_align.outputAligned.link(sync_node.inputs["depth_aligned"])
            else:
                stereo.depth.link(sync_node.inputs["depth_aligned"])

            link_record_metadata_node(pipeline, demux.outputs["depth_aligned"], f"{output_path}_ALIGNED_DEPTH")

            target_camera = cameras[target_camera_name]
            if not rectified:
                target_camera_output = target_camera.requestOutput(
                    target_camera_size,
                    fps=fps,
                    enableUndistortion=target_camera_undistortion,
                    resizeMode=target_camera_resize_mode,
                )
            else:
                if target_camera_name == "CAM_B":
                    target_camera_output = stereo.rectifiedLeft
                else:
                    target_camera_output = stereo.rectifiedRight

            target_camera_output.link(sync_node.inputs["target_camera"])
            rectified_suffix = "_R" if rectified else ""
            link_record_video_node(
                pipeline,
                demux.outputs["target_camera"],
                f"{output_path}_{target_camera_name}{rectified_suffix}_target",
                fps=fps,
            )

            sync_output_queue = sync_node.out.createOutputQueue(maxSize=1, blocking=True)
            print(f"Writing ArUco points to: {ref_points_mcap_path}")

            pipeline.start()
            seq_num = 0

            while pipeline.isRunning():
                assert sync_output_queue is not None
                message_group = sync_output_queue.get()
                assert isinstance(message_group, dai.MessageGroup)
                assert message_group.isSynced(int(1_000_000_000 / fps * 0.5)), (
                    f"Sync node emitted unsynced group interval={message_group.getIntervalNs()}ns "
                    f"threshold={1_000_000_000 / fps * 0.5}ns"
                )

                reference_frame = message_group["ref_camera"]
                target_frame = message_group["target_camera"]
                depth_message = message_group["depth_aligned"]

                assert isinstance(reference_frame, dai.ImgFrame)
                assert isinstance(target_frame, dai.ImgFrame)
                assert isinstance(depth_message, dai.ImgFrame)

                depth_frame = depth_message.getFrame()
                reference_img = ensure_bgr(reference_frame.getCvFrame())
                target_img = ensure_bgr(target_frame.getCvFrame())

                reference_aruco_points = calculate_aruco(reference_img)
                target_aruco_points = calculate_aruco(target_img)

                reference_data_points = aruco_points_to_mcap_points_list(
                    reference_aruco_points,
                    frame_width=reference_img.shape[1],
                    frame_height=reference_img.shape[0],
                    depth_frame=depth_frame,
                )
                target_data_points = aruco_points_to_mcap_points_list(
                    target_aruco_points,
                    frame_width=target_img.shape[1],
                    frame_height=target_img.shape[0],
                    depth_frame=None,
                )

                write_aruco_points(
                    writer=ref_mcap_writer,
                    channel_id=ref_mcap_channel_id,
                    ts=to_us(reference_frame.getTimestampDevice()),
                    data_points=reference_data_points,
                    seq_num=seq_num,
                )
                write_aruco_points(
                    writer=target_mcap_writer,
                    channel_id=target_mcap_channel_id,
                    ts=to_us(target_frame.getTimestampDevice()),
                    data_points=target_data_points,
                    seq_num=seq_num,
                )
                seq_num += 1

                overlay = draw_aruco_points(reference_img, reference_aruco_points, spatial_points=reference_data_points)
                target_overlay = draw_aruco_points(target_img, target_aruco_points, spatial_points=target_data_points)
                cv2.imshow(f"reference_aruco_points_{reference_camera_name}", overlay)
                cv2.imshow(f"target_aruco_points_{target_camera_name}{rectified_suffix}", target_overlay)
                if cv2.waitKey(1) == ord("q"):
                    pipeline.stop()
                    break

                if seq_num > duration * fps:
                    pipeline.stop()

    finally:
        cv2.destroyAllWindows()
        if ref_mcap_writer is not None:
            ref_mcap_writer.finish()
        if target_mcap_writer is not None:
            target_mcap_writer.finish()
        if ref_mcap_file is not None:
            ref_mcap_file.close()
        if target_mcap_file is not None:
            target_mcap_file.close()


if __name__ == "__main__":
    main()
