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
ARUCO_DETECTOR: Any = None
ARUCO_REFINE = True

CARUCO_BOARD_SQUARES_X = 9
CARUCO_BOARD_SQUARES_Y = 12
CARUCO_SQUARE_LENGTH_M = 0.060
CARUCO_MARKER_LENGTH_M = 0.047


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
    record_video.setRecordVideoFile(Path(f"{output_path}.avi"))
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


def create_charuco_board(
    aruco_dict: Any,
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
):
    if squares_x <= 1 or squares_y <= 1:
        return None

    board = None
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(squares_x, squares_y, square_length_m, marker_length_m, aruco_dict)
    else:
        try:
            board = cv2.aruco.CharucoBoard((squares_y, squares_x), square_length_m, marker_length_m,  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        except Exception:
            return None

    if board is not None and hasattr(board, "setLegacyPattern"):
        try:
            board.setLegacyPattern(False)
        except cv2.error:
            pass

    return board


def detect_aruco_markers(gray: np.ndarray, refine_board: Any = None):
    if ARUCO_DETECTOR is not None:
        corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    if ids is None or len(ids) == 0:
        return corners, ids, rejected

    refined_corners, refined_ids = corners, ids
    if ARUCO_REFINE and refine_board is not None and hasattr(cv2.aruco, "refineDetectedMarkers"):
        try:
            refine_result = cv2.aruco.refineDetectedMarkers(gray, refine_board, corners, ids, rejected)
            if len(refine_result) >= 2:
                candidate_corners, candidate_ids = refine_result[0], refine_result[1]
                if candidate_ids is not None and len(candidate_ids) > 0:
                    refined_corners, refined_ids = candidate_corners, candidate_ids
        except cv2.error:
            pass

    return refined_corners, refined_ids, rejected


def detect_charuco_points_in_gray(
    gray: np.ndarray,
    charuco_board: Any,
    camera_matrix: Optional[np.ndarray] = None,
    distortion_coeffs: Optional[np.ndarray] = None,
):
    marker_corners, marker_ids, _ = detect_aruco_markers(gray, refine_board=charuco_board)
    if marker_ids is None or len(marker_ids) == 0:
        return None, None

    return detect_charuco_corners(
        gray,
        marker_corners,
        marker_ids,
        charuco_board,
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_coeffs,
    )


def detect_charuco_corners(
    gray: np.ndarray,
    marker_corners: Any,
    marker_ids: Any,
    charuco_board: Any,
    camera_matrix: Optional[np.ndarray] = None,
    distortion_coeffs: Optional[np.ndarray] = None,
):
    if marker_ids is None or len(marker_ids) == 0:
        return None, None

    if hasattr(cv2.aruco, "interpolateCornersCharuco"):
        calls: list[tuple[tuple[Any, ...], dict[str, Any]]] = [
            (
                (marker_corners, marker_ids, gray, charuco_board),
                {
                    "cameraMatrix": camera_matrix,
                    "distCoeffs": distortion_coeffs,
                    "minMarkers": 1,
                },
            ),
            (
                (marker_corners, marker_ids, gray, charuco_board, camera_matrix, distortion_coeffs),
                {"minMarkers": 1},
            ),
            ((marker_corners, marker_ids, gray, charuco_board), {"minMarkers": 1}),
            ((marker_corners, marker_ids, gray, charuco_board), {}),
        ]
        for args, kwargs in calls:
            try:
                _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(*args, **kwargs)
            except (TypeError, cv2.error):
                continue
            return charuco_corners, charuco_ids

    if hasattr(cv2.aruco, "CharucoDetector"):
        try:
            detector = cv2.aruco.CharucoDetector(charuco_board)
            charuco_corners, charuco_ids, _, _ = detector.detectBoard(gray)
            return charuco_corners, charuco_ids
        except cv2.error:
            return None, None

    return None, None


def get_charuco_board_corners(charuco_board: Any) -> np.ndarray:
    if hasattr(charuco_board, "getChessboardCorners"):
        corners = charuco_board.getChessboardCorners()
    else:
        corners = getattr(charuco_board, "chessboardCorners", None)
    if corners is None:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(corners, dtype=np.float32).reshape(-1, 3)


def estimate_caruco_board_pose(
    img: np.ndarray,
    transformation: dai.ImgTransformation,
    charuco_board: Any,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, Optional[np.ndarray]]]:
    if charuco_board is None:
        return None

    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    camera_matrix = np.asarray(transformation.getIntrinsicMatrix(), dtype=np.float64)
    if camera_matrix.shape != (3, 3):
        return None
    distortion_coeffs = np.asarray(transformation.getDistortionCoefficients(), dtype=np.float64).reshape(-1, 1)
    distortion_input = distortion_coeffs if distortion_coeffs.size > 0 else None

    charuco_corners, charuco_ids = detect_charuco_points_in_gray(
        gray,
        charuco_board,
        camera_matrix=camera_matrix,
        distortion_coeffs=distortion_input,
    )
    if charuco_corners is None or charuco_ids is None:
        return None

    return estimate_caruco_board_pose_from_detection(
        charuco_corners=np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2),
        charuco_ids=np.asarray(charuco_ids, dtype=np.int32).reshape(-1),
        transformation=transformation,
        charuco_board=charuco_board,
    )


def estimate_caruco_board_pose_from_detection(
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    transformation: dai.ImgTransformation,
    charuco_board: Any,
) -> Optional[tuple[np.ndarray, np.ndarray, np.ndarray, Optional[np.ndarray]]]:
    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    if ids.size == 0 or image_points.shape[0] == 0:
        return None

    camera_matrix = np.asarray(transformation.getIntrinsicMatrix(), dtype=np.float64)
    if camera_matrix.shape != (3, 3):
        return None
    distortion_coeffs = np.asarray(transformation.getDistortionCoefficients(), dtype=np.float64).reshape(-1, 1)
    distortion_input = distortion_coeffs if distortion_coeffs.size > 0 else None

    board_corners = get_charuco_board_corners(charuco_board)
    if board_corners.shape[0] == 0:
        return None

    valid_idx = np.flatnonzero((ids >= 0) & (ids < board_corners.shape[0]))
    if valid_idx.size < 4:
        return None

    object_points = board_corners[ids[valid_idx]]
    image_points = image_points[valid_idx]

    try:
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            camera_matrix,
            distortion_input,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=8.0,
            iterationsCount=150,
            confidence=0.995,
        )
    except cv2.error:
        return None

    if not success:
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                camera_matrix,
                distortion_input,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except cv2.error:
            return None
        if not success:
            return None
    elif inliers is not None and len(inliers) >= 6:
        try:
            inlier_idx = inliers.flatten()
            success, rvec, tvec = cv2.solvePnP(
                object_points[inlier_idx],
                image_points[inlier_idx],
                camera_matrix,
                distortion_input,
                rvec=rvec,
                tvec=tvec,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not success:
                return None
        except cv2.error:
            return None

    tvec_reshaped = np.asarray(tvec, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(tvec_reshaped)):
        return None
    return (
        np.asarray(rvec, dtype=np.float64).reshape(3),
        tvec_reshaped,
        camera_matrix,
        distortion_input,
    )


def calculate_reference_charuco_points_and_board_depth(
    img: np.ndarray,
    transformation: dai.ImgTransformation,
    charuco_board: Any,
) -> tuple[list[dict[str, float | int]], np.ndarray]:
    frame_height, frame_width = img.shape[:2]
    charuco_depth_frame = np.zeros((frame_height, frame_width), dtype=np.uint16)

    if charuco_board is None:
        return [], charuco_depth_frame

    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    charuco_corners, charuco_ids = detect_charuco_points_in_gray(
        gray,
        charuco_board,
    )
    if charuco_corners is None or charuco_ids is None:
        return [], charuco_depth_frame

    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    corners = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    if ids.size == 0 or corners.shape[0] == 0:
        return [], charuco_depth_frame
    if ids.size != corners.shape[0]:
        return [], charuco_depth_frame

    order = np.argsort(ids)
    points: list[dict[str, float | int]] = []
    for idx in order:
        points.append(
            {
                "x": float(corners[idx, 0]),
                "y": float(corners[idx, 1]),
                "aruco_id": -1,
                "marker_corner_index": -1,
                "charuco_corner_id": int(ids[idx]),
            }
        )

    pose = estimate_caruco_board_pose_from_detection(
        charuco_corners=corners,
        charuco_ids=ids,
        transformation=transformation,
        charuco_board=charuco_board,
    )
    if pose is None:
        print("CHARUCO_DEPTH pose unavailable from current detection; skipping board-depth override.")
        return points, charuco_depth_frame

    rvec, tvec, pose_camera_matrix, pose_distortion_coeffs = pose
    rotation_matrix, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    tvec_vec = np.asarray(tvec, dtype=np.float64).reshape(3)
    board_corners = get_charuco_board_corners(charuco_board)
    written_points = 0
    for point in points:
        x_px = int(round(float(point["x"])))
        y_px = int(round(float(point["y"])))
        charuco_corner_id = int(point["charuco_corner_id"])
        if x_px < 0 or y_px < 0 or x_px >= frame_width or y_px >= frame_height:
            continue

        depth_mm: Optional[float] = None
        if 0 <= charuco_corner_id < board_corners.shape[0]:
            obj_point = np.asarray(board_corners[charuco_corner_id], dtype=np.float64).reshape(3)
            point_cam = rotation_matrix @ obj_point + tvec_vec
            depth_mm = float(point_cam[2] * 1000.0)

        if depth_mm is None or not np.isfinite(depth_mm) or depth_mm <= 0.0:
            depth_mm = board_plane_depth_mm_at_pixel(
                x_px=x_px,
                y_px=y_px,
                rvec=rvec,
                tvec=tvec,
                camera_matrix=pose_camera_matrix,
                distortion_coeffs=pose_distortion_coeffs,
            )

        if depth_mm is None or not np.isfinite(depth_mm) or depth_mm <= 0.0:
            continue
        depth_value = int(round(depth_mm))
        max_depth_value = int(np.iinfo(charuco_depth_frame.dtype).max)
        depth_value = min(depth_value, max_depth_value)
        charuco_depth_frame[y_px, x_px] = depth_value
        written_points += 1

    print(f"CHARUCO_DEPTH frame wrote {written_points}/{len(points)} points")
    return points, charuco_depth_frame


def pixel_ray_from_intrinsics(
    x_px: int,
    y_px: int,
    camera_matrix: np.ndarray,
    distortion_coeffs: Optional[np.ndarray],
) -> Optional[np.ndarray]:
    try:
        undistorted = cv2.undistortPoints(
            np.array([[[float(x_px), float(y_px)]]], dtype=np.float64),
            camera_matrix,
            distortion_coeffs,
        )
    except cv2.error:
        return None

    x_n = float(undistorted[0, 0, 0])
    y_n = float(undistorted[0, 0, 1])
    if not np.isfinite(x_n) or not np.isfinite(y_n):
        return None
    return np.array([x_n, y_n, 1.0], dtype=np.float64)


def board_plane_depth_mm_at_pixel(
    x_px: int,
    y_px: int,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    distortion_coeffs: Optional[np.ndarray],
) -> Optional[float]:
    ray = pixel_ray_from_intrinsics(x_px, y_px, camera_matrix, distortion_coeffs)
    if ray is None:
        return None

    rotation_matrix, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    plane_normal = rotation_matrix[:, 2]
    plane_point = tvec.reshape(3)

    denominator = float(np.dot(plane_normal, ray))
    if abs(denominator) < 1e-9:
        return None

    ray_scale = float(np.dot(plane_normal, plane_point)) / denominator
    if not np.isfinite(ray_scale) or ray_scale <= 0.0:
        return None

    point_cam = ray * ray_scale
    depth_mm = float(point_cam[2] * 1000.0)
    if not np.isfinite(depth_mm) or depth_mm <= 0.0:
        return None
    return depth_mm


def calculate_charuco(img: np.ndarray, charuco_board: Any) -> list[dict[str, float | int]]:
    if charuco_board is None:
        return []

    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img

    charuco_corners, charuco_ids = detect_charuco_points_in_gray(gray, charuco_board)
    if charuco_corners is None or charuco_ids is None:
        return []

    ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    corners = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 2)
    if ids.size == 0 or corners.shape[0] == 0:
        return []
    if ids.size != corners.shape[0]:
        return []

    order = np.argsort(ids)
    points: list[dict[str, float | int]] = []
    for idx in order:
        charuco_corner_id = int(ids[idx])
        x = float(corners[idx, 0])
        y = float(corners[idx, 1])
        points.append(
            {
                "x": x,
                "y": y,
                "aruco_id": -1,
                "marker_corner_index": -1,
                "charuco_corner_id": charuco_corner_id,
            }
        )
    return points


def aruco_points_to_mcap_points_list(
    charuco_points: Iterable[dict[str, float | int]],
    frame_width: int,
    frame_height: int,
    depth_frame: np.ndarray | None,
    depth_source_label: Optional[str] = None,
) -> list[dict[str, Optional[float]]]:
    width = max(1, int(frame_width))
    height = max(1, int(frame_height))
    points_spatial: list[dict[str, Optional[float]]] = []
    valid_depth_points = 0
    for point in charuco_points:
        # Preserve detected ChArUco subpixel coordinates in recorded points.
        x_px_float = float(point["x"])
        y_px_float = float(point["y"])
        x_px_sample = int(round(x_px_float))
        y_px_sample = int(round(y_px_float))
        aruco_id = int(point["aruco_id"])
        marker_corner_index = int(point["marker_corner_index"])
        charuco_corner_id = int(point["charuco_corner_id"])

        sampled_depth_mm = sample_depth_mm(depth_frame, x_px_sample, y_px_sample, radius=1)
        depth_mm_value = float(sampled_depth_mm) if sampled_depth_mm is not None else None

        if depth_mm_value is None:
            points_spatial.append(
                {
                    "pixel_x": x_px_float / width,
                    "pixel_y": y_px_float / height,
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
                "pixel_x": x_px_float / width,
                "pixel_y": y_px_float / height,
                "depth_mm": float(depth_mm_value),
                "valid": True,
                "aruco_id": aruco_id,
                "marker_corner_index": marker_corner_index,
                "charuco_corner_id": charuco_corner_id,
            }
        )
        valid_depth_points += 1

    if depth_frame is not None and depth_source_label is not None:
        print(f"{depth_source_label} sampled valid {valid_depth_points}/{len(points_spatial)} points into MCAP")
    return points_spatial


def draw_charuco_points(
    frame: np.ndarray,
    charuco_points: list[dict[str, float | int]],
    spatial_points: Optional[list[dict[str, Optional[float]]]] = None,
) -> np.ndarray:
    vis = ensure_bgr(frame).copy()

    for idx, point in enumerate(charuco_points):
        x = float(point["x"])
        y = float(point["y"])
        color = (0, 255, 0)
        if spatial_points is not None and idx < len(spatial_points):
            color = (0, 255, 0) if spatial_points[idx].get("valid", False) else (0, 0, 255)
        cv2.circle(vis, (int(round(x)), int(round(y))), 4, color, -1)

    cv2.putText(
        vis,
        f"charuco_corners:{len(charuco_points)}",
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 180, 0),
        1,
        cv2.LINE_AA,
    )
    return vis

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



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output-path", default="capture/default/", help="Output path prefix")
    parser.add_argument("-d", "--duration", type=int, default=10, help="Duration in seconds")
    parser.add_argument("--fps", type=int, default=15, help="FPS for cameras and recording")
    parser.add_argument(
        "-rc",
        "--reference-camera",
        type=str,
        default="CAM_A",
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Reference camera used for ChArUco corner detection.",
    )
    parser.add_argument(
        "-tc",
        "--target-camera",
        type=str,
        default="CAM_B",
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Target camera to save ChArUco points from.",
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
        default="DICT_4X4_250",
        help="OpenCV ArUco dictionary used by the ChArUco board (for example DICT_4X4_250).",
    )
    parser.add_argument(
        "--refine-markers",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Run board-aware refine step after marker detection for ChArUco interpolation.",
    )
    parser.add_argument(
        "-cbd",
        "--caruco-board-depth",
        "--charuco-board-depth",
        dest="caruco_board_depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Estimate reference depth from ChArUco board pose (9x12 squares, 60mm/47mm) instead of stereo depth.",
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
    use_caruco_board_depth = args.caruco_board_depth

    global ARUCO_DICT, ARUCO_PARAMS, ARUCO_DETECTOR, ARUCO_REFINE
    ARUCO_DICT = resolve_aruco_dictionary(args.aruco_dictionary)
    ARUCO_PARAMS = create_aruco_parameters()
    ARUCO_DETECTOR = create_aruco_detector(ARUCO_DICT, ARUCO_PARAMS)
    ARUCO_REFINE = args.refine_markers

    caruco_board = create_charuco_board(
        ARUCO_DICT,
        squares_x=CARUCO_BOARD_SQUARES_X,
        squares_y=CARUCO_BOARD_SQUARES_Y,
        square_length_m=CARUCO_SQUARE_LENGTH_M,
        marker_length_m=CARUCO_MARKER_LENGTH_M,
    )
    if caruco_board is None:
        raise RuntimeError("Failed to create ChArUco board. Check OpenCV ArUco support in your environment.")

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
            "caruco_board_squares_x": CARUCO_BOARD_SQUARES_X,
            "caruco_board_squares_y": CARUCO_BOARD_SQUARES_Y,
            "caruco_square_length_m": CARUCO_SQUARE_LENGTH_M,
            "caruco_marker_length_m": CARUCO_MARKER_LENGTH_M,
            "depth_source": "caruco_board" if use_caruco_board_depth else "stereo_aligned_depth",
        }
    )
    print(
        "Reference depth source: "
        + ("charuco_board (-cbd enabled)" if use_caruco_board_depth else "stereo_aligned_depth (-cbd disabled)")
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
            "caruco_board_squares_x": CARUCO_BOARD_SQUARES_X,
            "caruco_board_squares_y": CARUCO_BOARD_SQUARES_Y,
            "caruco_square_length_m": CARUCO_SQUARE_LENGTH_M,
            "caruco_marker_length_m": CARUCO_MARKER_LENGTH_M,
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
    
    device = dai.Device()
    print("Sensor names:", device.getCameraSensorNames())

    # Detailed feature info, including supported configs/resolutions
    for cam in device.getConnectedCameraFeatures():
        print(f"\nSocket: {cam.socket}")
        print(f"Sensor: {cam.sensorName}")
        print(f"Max/native: {cam.width}x{cam.height}")
        print(f"Supported types: {cam.supportedTypes}")

        # Each config is a supported sensor mode
        for cfg in cam.configs:
            print("  ", cfg)

    try:
        with dai.Pipeline(device) as pipeline:

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

            camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            cameras = {"CAM_A": camera_a, "CAM_B": camera_b, "CAM_C": camera_c}

            calib_path = Path(f"{output_path}calib.json")
            calib_path.parent.mkdir(parents=True, exist_ok=True)
            device.readCalibration().eepromToJsonFile(str(calib_path))
            print(f"Saved calibration to: {calib_path}")

            reference_camera = cameras[reference_camera_name]

            # if not rectified:
            camera_c_output = camera_c.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )
            camera_b_output = camera_b.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )
            # else:
            #     camera_c_output = camera_c.requestOutput(
            #         target_camera_size,
            #         fps=fps,
            #         enableUndistortion=target_camera_undistortion,
            #         resizeMode=target_camera_resize_mode,
            #     )
            #     camera_b_output = camera_b.requestOutput(
            #         target_camera_size,
            #         fps=fps,
            #         enableUndistortion=target_camera_undistortion,
            #         resizeMode=target_camera_resize_mode,
            #     )

            reference_camera_output = reference_camera.requestOutput(
                ref_camera_size,
                fps=fps,
                enableUndistortion=ref_camera_undistortion,
                resizeMode=ref_camera_resize_mode,
            )
            # reference_camera_output = reference_camera.requestFullResolutionOutput()
            

            stereo = pipeline.create(dai.node.StereoDepth).build(camera_b_output, camera_c_output, presetMode=dai.node.StereoDepth.PresetMode.FAST_DENSITY)
            # reference_camera_output.link(stereo.inputAlignTo)

            # reference_camera_output = stereo.rectifiedRight
            reference_camera_output.link(sync_node.inputs["ref_camera"])
            link_record_video_node(
                pipeline,
                demux.outputs["ref_camera"],
                f"{output_path}{reference_camera_name}_reference",
                fps=fps,
            )

            if device.getPlatform() == dai.Platform.RVC4:
                
                image_align = pipeline.create(dai.node.ImageAlign)
                stereo.depth.link(image_align.input)
                reference_camera_output.link(image_align.inputAlignTo)
                image_align.outputAligned.link(sync_node.inputs["depth_aligned"])
            else:
                stereo.depth.link(sync_node.inputs["depth_aligned"])

            link_record_metadata_node(pipeline, demux.outputs["depth_aligned"], f"{output_path}_ALIGNED_DEPTH")

            target_camera = cameras[target_camera_name]
            # if not rectified:
            target_camera_output = target_camera.requestOutput(
                target_camera_size,
                fps=fps,
                enableUndistortion=target_camera_undistortion,
                resizeMode=target_camera_resize_mode,
            )
            # else:
            #     if target_camera_name == "CAM_B":
            #         target_camera_output = stereo.rectifiedLeft
            #     else:
            #         target_camera_output = stereo.rectifiedRight

            # target_camera_output = camera_b_output
            target_camera_output.link(sync_node.inputs["target_camera"])
            
            # target_fuller_res = target_camera.requestOutput((1920, 1080), fps=fps, enableUndistortion=target_camera_undistortion, resizeMode=dai.ImgResizeMode.CROP)
            # imgManip = pipeline.create(dai.node.ImageManip)
            # imgManip.initialConfig.addFlipHorizontal()
            # imgManip.initialConfig.addFlipVertical()
            # imgManip.initialConfig.addRotateDeg(9)
            # imgManip.initialConfig.setOutputSize(1478, 600, dai.ImageManipConfig.ResizeMode.STRETCH)
            # imgManip.setMaxOutputFrameSize(1920*1080*3)
            # target_fuller_res.link(imgManip.inputImage)
            
            # imgManip.out.link(sync_node.inputs["target_camera"])

            rectified_suffix = "_R" if rectified else ""
            link_record_video_node(
                pipeline,
                demux.outputs["target_camera"],
                f"{output_path}_{target_camera_name}{rectified_suffix}_target",
                fps=fps,
            )
            pipeline.enableHolisticReplay("recordings/recording_2026-03-11_16-51-20.tar")

            sync_output_queue = sync_node.out.createOutputQueue(maxSize=1, blocking=True)
            print(f"Writing ChArUco points to: {ref_points_mcap_path}")

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

                # printTransformation("Reference camera", reference_frame.getTransformation())
                # printTransformation("Target camera", target_frame.getTransformation())

                reference_transformation = reference_frame.getTransformation()
                reference_charuco_points, reference_charuco_depth_frame = calculate_reference_charuco_points_and_board_depth(
                    reference_img,
                    transformation=reference_transformation,
                    charuco_board=caruco_board,
                )
                target_charuco_points = calculate_charuco(target_img, charuco_board=caruco_board)

                if use_caruco_board_depth:
                    depth_frame = reference_charuco_depth_frame.copy()

                reference_data_points = aruco_points_to_mcap_points_list(
                    reference_charuco_points,
                    frame_width=reference_img.shape[1],
                    frame_height=reference_img.shape[0],
                    depth_frame=depth_frame,
                    depth_source_label="CHARUCO_DEPTH" if use_caruco_board_depth else "STEREO_DEPTH",
                )
                target_data_points = aruco_points_to_mcap_points_list(
                    target_charuco_points,
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

                overlay = draw_charuco_points(
                    reference_img, reference_charuco_points, spatial_points=reference_data_points
                )
                target_overlay = draw_charuco_points(
                    target_img, target_charuco_points, spatial_points=target_data_points
                )
                cv2.imshow(f"target_charuco_points_{target_camera_name}{rectified_suffix}", target_overlay)
                cv2.imshow(f"reference_charuco_points_{reference_camera_name}", overlay)
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
