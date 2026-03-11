import argparse
import json
import os
import signal
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable, Optional
from mcap.writer import Writer

import cv2
import depthai as dai
import numpy as np


def normalize_point2f(point: dai.Point2f, width: int, height: int) -> dict[str, float]:
    return {"x": float(point.x) / float(width), "y": float(point.y) / float(height)}


def april_tag_to_corner_pixels(april_tag_message: dai.AprilTags) -> list[tuple[float, float]]:
    corners: list[tuple[float, float]] = []
    for tag in april_tag_message.aprilTags:
        corners.append((float(tag.topLeft.x), float(tag.topLeft.y)))
        corners.append((float(tag.topRight.x), float(tag.topRight.y)))
        corners.append((float(tag.bottomRight.x), float(tag.bottomRight.y)))
        corners.append((float(tag.bottomLeft.x), float(tag.bottomLeft.y)))
    return corners


def april_tag_to_points(april_tag_message: dai.AprilTags, width: int, height: int) -> list[dict[str, float]]:
    data_points: list[dict[str, float]] = []
    for x, y in april_tag_to_corner_pixels(april_tag_message):
        data_points.append(normalize_point2f(dai.Point2f(x, y), width, height))
    return data_points


def draw_apriltag_points(
    rgb_frame: np.ndarray,
    april_tag_message: dai.AprilTags,
    spatial_points: Optional[list[dict[str, Optional[float]]]] = None,
) -> np.ndarray:
    vis = rgb_frame.copy()
    spatial_idx = 0
    for tag in april_tag_message.aprilTags:
        corners = [tag.topLeft, tag.topRight, tag.bottomRight, tag.bottomLeft]
        pts = np.array([[int(round(c.x)), int(round(c.y))] for c in corners], dtype=np.int32)
        cv2.polylines(vis, [pts], isClosed=True, color=(255, 180, 0), thickness=2)

        center_x = int(round(sum(c.x for c in corners) / 4.0))
        center_y = int(round(sum(c.y for c in corners) / 4.0))
        cv2.putText(vis, f"id:{tag.id}", (center_x + 4, center_y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 1, cv2.LINE_AA)

        for px, py in pts:
            color = (0, 255, 0)
            if spatial_points is not None and spatial_idx < len(spatial_points):
                color = (0, 255, 0) if spatial_points[spatial_idx].get("valid", False) else (0, 0, 255)
            cv2.circle(vis, (int(px), int(py)), 4, color, -1)
            spatial_idx += 1
    return vis


def sample_depth_mm(depth_frame: np.ndarray, x: int, y: int, radius: int = 1) -> Optional[int]:
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


def april_tag_to_spatial_points(
    april_tag_message: dai.AprilTags,
    depth_frame: np.ndarray,
    intrinsic_matrix: Iterable[Iterable[float]],
) -> list[dict[str, Optional[float]]]:
    intr = np.asarray(intrinsic_matrix, dtype=np.float64)
    fx = float(intr[0, 0])
    fy = float(intr[1, 1])
    cx = float(intr[0, 2])
    cy = float(intr[1, 2])

    points_spatial: list[dict[str, Optional[float]]] = []
    for px, py in april_tag_to_corner_pixels(april_tag_message):
        x_px = int(round(px))
        y_px = int(round(py))
        depth_mm = sample_depth_mm(depth_frame, x_px, y_px, radius=1)
        if depth_mm is None:
            points_spatial.append(
                {
                    "pixel_x": x_px,
                    "pixel_y": y_px,
                    "x": None,
                    "y": None,
                    "z": None,
                    "depth_mm": None,
                    "valid": False,
                }
            )
            continue

        x_mm = (float(x_px) - cx) * float(depth_mm) / fx
        y_mm = (float(y_px) - cy) * float(depth_mm) / fy
        points_spatial.append(
            {
                "pixel_x": x_px,
                "pixel_y": y_px,
                "x": x_mm,
                "y": y_mm,
                "z": float(depth_mm),
                "depth_mm": float(depth_mm),
                "valid": True,
            }
        )

    return points_spatial


def to_ns(ts: Any) -> int:
    if ts is None:
        return 0
    if isinstance(ts, timedelta):
        return int(ts.total_seconds() * 1_000_000_000)
    if isinstance(ts, datetime):
        if ts.tzinfo is None:
            ts = ts.replace(tzinfo=timezone.utc)
        return int(ts.timestamp() * 1_000_000_000)
    if isinstance(ts, int):
        return ts
    if isinstance(ts, float):
        return int(ts * 1_000_000_000)
    if hasattr(ts, "total_seconds"):
        return int(ts.total_seconds() * 1_000_000_000)
    raise TypeError(f"Unsupported timestamp type: {type(ts)}")


def select_ns(primary: int, fallback: int) -> int:
    return primary if primary > 0 else fallback


def create_mcap_writer(output_path: Path, spatial_points: bool):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mcap_fp = output_path.open("wb")
    writer = Writer(mcap_fp)
    writer.start()

    points_field_name = "points_spatial_mm" if spatial_points else "points_normalized"
    points_item_schema: dict[str, Any]
    if spatial_points:
        points_item_schema = {
            "type": "object",
            "properties": {
                "pixel_x": {"type": "integer"},
                "pixel_y": {"type": "integer"},
                "x": {"type": ["number", "null"]},
                "y": {"type": ["number", "null"]},
                "z": {"type": ["number", "null"]},
                "depth_mm": {"type": ["number", "null"]},
                "valid": {"type": "boolean"},
            },
            "required": ["pixel_x", "pixel_y", "x", "y", "z", "depth_mm", "valid"],
        }
    else:
        points_item_schema = {
            "type": "object",
            "properties": {"x": {"type": "number"}, "y": {"type": "number"}},
            "required": ["x", "y"],
        }

    schema = {
        "type": "object",
        "properties": {
            "reference_camera": {"type": "string"},
            "april_sequence_num": {"type": "integer"},
            "rgb_sequence_num": {"type": "integer"},
            "depth_sequence_num": {"type": "integer"},
            "timestamp_host_ns": {"type": "integer"},
            "timestamp_device_ns": {"type": "integer"},
            "depth_timestamp_host_ns": {"type": "integer"},
            "depth_timestamp_device_ns": {"type": "integer"},
            points_field_name: {
                "type": "array",
                "items": points_item_schema,
            },
        },
        "required": [
            "reference_camera",
            "april_sequence_num",
            "rgb_sequence_num",
            "timestamp_host_ns",
            "timestamp_device_ns",
            points_field_name,
        ],
    }
    schema_id = writer.register_schema(
        name="depthai.apriltag.points",
        encoding="jsonschema",
        data=json.dumps(schema, separators=(",", ":")).encode("utf-8"),
    )
    channel_id = writer.register_channel(topic="apriltag/points_spatial" if spatial_points else "apriltag/points", message_encoding="json", schema_id=schema_id)
    return mcap_fp, writer, channel_id


def link_record_node(
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


def ensure_reference_camera(reference_camera: str, save_depth: bool) -> None:
    if save_depth:
        return
    depth_only = {"CAM_B", "CAM_C", "CAM_B_R", "CAM_C_R"}
    if reference_camera in depth_only:
        raise ValueError(f"Reference camera '{reference_camera}' requires --depth True")


def write_apriltag_points(
    writer,
    channel_id: int,
    reference_camera: str,
    in_april_message: dai.AprilTags,
    in_rgb_message: dai.ImgFrame,
    data_points: Iterable[dict[str, Any]],
    data_points_field: str,
    in_depth_message: Optional[dai.ImgFrame] = None,
) -> None:
    april_seq = int(in_april_message.getSequenceNum())
    rgb_seq = int(in_rgb_message.getSequenceNum())

    april_host_ns = to_ns(in_april_message.getTimestamp())
    april_device_ns = to_ns(in_april_message.getTimestampDevice())
    rgb_host_ns = to_ns(in_rgb_message.getTimestamp())
    rgb_device_ns = to_ns(in_rgb_message.getTimestampDevice())

    log_time_ns = max(0, select_ns(april_host_ns, rgb_host_ns))
    publish_time_ns = max(0, select_ns(april_device_ns, rgb_device_ns))

    payload = {
        "reference_camera": reference_camera,
        "april_sequence_num": april_seq,
        "rgb_sequence_num": rgb_seq,
        "timestamp_host_ns": log_time_ns,
        "timestamp_device_ns": publish_time_ns,
        data_points_field: list(data_points),
    }
    if in_depth_message is not None:
        payload["depth_sequence_num"] = int(in_depth_message.getSequenceNum())
        payload["depth_timestamp_host_ns"] = to_ns(in_depth_message.getTimestamp())
        payload["depth_timestamp_device_ns"] = to_ns(in_depth_message.getTimestampDevice())

    writer.add_message(
        channel_id=channel_id,
        log_time=log_time_ns,
        publish_time=publish_time_ns,
        sequence=april_seq if april_seq >= 0 else rgb_seq,
        data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output-path", default="capture/default/", help="Output path prefix")
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        default="CAM_A",
        choices=["CAM_A", "CAM_C_R"],
        help="Reference camera used for AprilTag detection.",
    )
    parser.add_argument("-d", "--duration", type=int, default=10, help="Duration in seconds")
    parser.add_argument("--size", type=tuple, default=(1280, 720), help="Resolution for virtual camera E as (width, height)")
    parser.add_argument("--distortion", type=bool, default=True, help="Enable undistortion for RGB camera")
    parser.add_argument("--depth", type=bool, default=True, help="Enable depth outputs")
    parser.add_argument("--visualize", type=bool, default=True, help="Show OpenCV overlay of points on RGB frame")
    args = parser.parse_args()

    output_path = args.output_path
    duration = args.duration
    size = args.size
    enable_undistortion = args.distortion
    save_depth = args.depth
    visualize = args.visualize
    reference_camera = args.camera
    ensure_reference_camera(reference_camera, save_depth)

    mcap_file = None
    mcap_writer = None
    mcap_channel_id = None
    sync_output_queue = None
    fps = 30
    try:
        with dai.Pipeline() as pipeline:
            def signal_handler(sig, frame):
                print("Interrupted, stopping the pipeline")
                pipeline.stop()

            signal.signal(signal.SIGINT, signal_handler)
            syncNode = pipeline.create(dai.node.Sync)
            syncNode.setRunOnHost(False)
            syncNode.setSyncThreshold(timedelta(milliseconds=1000/ fps * 0.5))
            syncNode.setSyncAttempts(-1)

            camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            camera_a_output = camera_a.requestOutput(
                (1280, 720), type=dai.ImgFrame.Type.NV12, fps=fps, enableUndistortion=enable_undistortion
            )
            camera_a_output.link(syncNode.inputs["cam_a"])

            demux = pipeline.create(dai.node.MessageDemux)
            syncNode.out.link(demux.input)
            link_record_node(pipeline, demux.outputs["cam_a"], f"{output_path}_CAM_A", fps=fps)
            
            if save_depth:
                camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                camera_b_output = camera_b.requestFullResolutionOutput(fps=fps)
                camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                camera_c_output = camera_c.requestFullResolutionOutput(fps=fps)

                stereo = pipeline.create(dai.node.StereoDepth)
                stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
                camera_b_output.link(stereo.left)
                camera_c_output.link(stereo.right)

                camera_c_rectified_output = stereo.rectifiedRight
                camera_c_rectified_output.link(syncNode.inputs["rectified_right"])

                link_record_node(pipeline, demux.outputs["rectified_right"], f"{output_path}_CAM_C_R", fps=fps)

                image_align = pipeline.create(dai.node.ImageAlign)
                reference_camera_output = {
                    "CAM_A": camera_a_output,
                    "CAM_C_R": camera_c_rectified_output,
                }[reference_camera]
                reference_camera_output.link(image_align.inputAlignTo)
                stereo.depth.link(image_align.input)

                depth_output = image_align.outputAligned
                depth_output.link(syncNode.inputs["depth_aligned"])

                link_record_metadata_node(pipeline, demux.outputs["depth_aligned"], f"{output_path}_DEPTH_ALIGNED")

            else:
                reference_camera_output = {
                    "CAM_A": camera_a_output,
                }[reference_camera]

            april_tag_node = pipeline.create(dai.node.AprilTag)
            reference_camera_output.link(april_tag_node.inputImage)
            april_tag_node.out.link(syncNode.inputs["april_tags"])

            reference_camera_key = "cam_a" if reference_camera == "CAM_A" else "rectified_right"
            target_camera_key = "rectified_right" if reference_camera == "CAM_A" else "cam_a"
            
            sync_output_queue = syncNode.out.createOutputQueue(maxSize=1, blocking=True)

            april_points_mcap_path = Path(f"{output_path}_{reference_camera}__APRIL_POINTS.mcap")
            mcap_file, mcap_writer, mcap_channel_id = create_mcap_writer(april_points_mcap_path, spatial_points=save_depth)
            print(f"Writing AprilTag points to: {april_points_mcap_path}")
            if visualize and os.environ.get("DISPLAY", "") == "":
                print("DISPLAY not set, disabling visualization")
                visualize = False

            pipeline.start()
            recording_start_time = time.monotonic()
            while pipeline.isRunning():
                assert sync_output_queue is not None
                message_group = sync_output_queue.get()
                assert isinstance(message_group, dai.MessageGroup)
                assert message_group.isSynced(int(1000000000 / fps * 0.5)), (
                    f"Sync node emitted unsynced group interval={message_group.getIntervalNs()}ns "
                    f"threshold={1000000000 / fps * 0.5}ns"
                )

                reference_frame = message_group[reference_camera_key]
                target_frame = message_group[target_camera_key]
                in_april_message = message_group["april_tags"]

                assert isinstance(in_april_message, dai.AprilTags)
                assert isinstance(reference_frame, dai.ImgFrame)
                assert isinstance(target_frame, dai.ImgFrame)

                reference_device_ns = to_ns(reference_frame.getTimestampDevice())
                target_device_ns = to_ns(target_frame.getTimestampDevice())
                device_delta_ns = abs(reference_device_ns - target_device_ns)
                assert device_delta_ns <= int(1_000_000_000 / fps * 0.5), (
                    f"Reference/target device timestamp mismatch: "
                    f"ref={reference_device_ns}, target={target_device_ns}, "
                    f"delta_ms={device_delta_ns / 1_000_000.0:.3f}"
                )

                in_depth_message = None
                if save_depth:
                    in_depth_message = message_group["depth_aligned"]
                    assert isinstance(in_depth_message, dai.ImgFrame)
                    depth_frame = in_depth_message.getFrame()
                    intrinsics = reference_frame.getTransformation().getIntrinsicMatrix()
                    data_points = april_tag_to_spatial_points(in_april_message, depth_frame, intrinsics)
                    points_field = "points_spatial_mm"
                else:
                    frame_width = reference_frame.getWidth()
                    frame_height = reference_frame.getHeight()
                    data_points = april_tag_to_points(in_april_message, frame_width, frame_height)
                    points_field = "points_normalized"

                if visualize:
                    rgb_frame = reference_frame.getCvFrame()
                    overlay = draw_apriltag_points(
                        rgb_frame,
                        in_april_message,
                        spatial_points=data_points if points_field == "points_spatial_mm" else None,
                    )
                    cv2.imshow("save_data_points", overlay)
                    if cv2.waitKey(1) == ord("q"):
                        pipeline.stop()
                        break

                write_apriltag_points(
                    writer=mcap_writer,
                    channel_id=mcap_channel_id,
                    reference_camera=reference_camera,
                    in_april_message=in_april_message,
                    in_rgb_message=reference_frame,
                    data_points=data_points,
                    data_points_field=points_field,
                    in_depth_message=in_depth_message,
                )

                if duration > 0 and (time.monotonic() - recording_start_time) >= duration:
                    pipeline.stop()
    finally:
        if visualize:
            cv2.destroyAllWindows()
        if mcap_writer is not None:
            mcap_writer.finish()
        if mcap_file is not None:
            mcap_file.close()


if __name__ == "__main__":
    main()
