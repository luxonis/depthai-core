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


def april_tag_to_mcap_points_list(
    april_tag_message: dai.AprilTags,
    depth_frame: np.ndarray | None,
    ) -> list[dict[str, Optional[float]]]:
    (w, h) = (1, 1)
    if depth_frame is not None:
        w, h = depth_frame.shape[1], depth_frame.shape[0]

    points_spatial: list[dict[str, Optional[float]]] = []
    for px, py in april_tag_to_corner_pixels(april_tag_message):
        x_px = int(round(px))
        y_px = int(round(py))

        depth_mm = sample_depth_mm(depth_frame, x_px, y_px, radius=1)
        if depth_mm is None:
            points_spatial.append(
                {
                    "pixel_x": x_px / w,
                    "pixel_y": y_px / h,
                    "depth_mm": None,
                    "valid": False,
                }
            )
            continue

        points_spatial.append(
            {
                "pixel_x": x_px / w,
                "pixel_y": y_px / h,
                "depth_mm": float(depth_mm),
                "valid": True,
            }
        )

    return points_spatial


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


def select_us(primary: int, fallback: int) -> int:
    return primary if primary > 0 else fallback

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

    points_item_schema: dict[str, Any]
    points_item_schema = {
            "type": "object",
            "properties": {
                "pixel_x": {"type": "number"},
                "pixel_y": {"type": "number"},
                "depth_mm": {"type": ["number", "null"]},
                "valid": {"type": "boolean"},
            },
            "required": ["pixel_x", "pixel_y", "depth_mm", "valid"],
    }
    
    schema = {
        "type": "object",
        "properties": {
            "timestamp_us" : {"type": "integer"},
            "points": {
                "type": "array",
                "items": points_item_schema,
            },
        },
        "required": [
            "timestamp_us",
            "points",
        ],
    }
    schema_id = writer.register_schema(
        name="depthai.apriltag.points",
        encoding="jsonschema",
        data=json.dumps(schema, separators=(",", ":")).encode("utf-8"),
    )
    channel_id = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema_id)
    return mcap_fp, writer, channel_id

def write_apriltag_points(
    writer: Writer,
    channel_id: int,
    ts: int,
    data_points: Iterable[dict[str, Any]],
    seq_num: int
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

def create_writer_metadata(point_type: str, socket: str, undistortion_enabled: bool, size: tuple[int, int], resize_mode: str, rectified: bool):
    return {
        "point_type": point_type,
        "socket": socket,
        "undistortion_enabled": undistortion_enabled,
        "size": {
            "width": size[0],
            "height": size[1],
        },
        "resize_mode": resize_mode,
        "rectified": rectified,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-o", "--output-path", default="capture/default/", help="Output path prefix")
    parser.add_argument("-d", "--duration", type=int, default=10, help="Duration in seconds")
    parser.add_argument("--visualize", action=argparse.BooleanOptionalAction, default=False, help="Show OpenCV overlay of points on RGB frame")
    parser.add_argument("-manip", "--manip", action=argparse.BooleanOptionalAction, default=False, help="Enable ImageManip node that rotates and enlarges the reference camera output before AprilTag detection")
    parser.add_argument("--fps", type=int, default=30, help="FPS for cameras and recording")
    parser.add_argument(
        "-rc",
        "--reference-camera",
        type=str,
        default="CAM_A",
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Reference camera used for AprilTag detection.",
    )
    parser.add_argument(
        "-tc", 
        "--target-camera", 
        type=str, 
        default="CAM_B", 
        choices=["CAM_A", "CAM_B", "CAM_C"],
        help="Camera to align depth to if --depth is enabled.")
    parser.add_argument("-r", "--rectified", "--rectification", dest="rectified", action=argparse.BooleanOptionalAction, default=False, help="Use rectified target camera output. Only applicable if target camera is CAM_B or CAM_C.")
    parser.add_argument("-rcs", "--reference-camera-size", nargs=2, type=int, metavar=("W", "H"), default=(1280, 720), help="Resolution of the reference camera as W H")
    parser.add_argument("-tcs", "--target-camera-size", nargs=2, type=int, metavar=("W", "H"), default=(1280, 800), help="Resolution of the target camera as W H")
    parser.add_argument("-rcd", "--reference-camera-undistortion", action=argparse.BooleanOptionalAction, default=False, help="Enable undistortion for reference camera")
    parser.add_argument("-tcd", "--target-camera-undistortion", action=argparse.BooleanOptionalAction, default=False, help="Enable undistortion for target camera if different from reference camera")
    parser.add_argument("-rcrm", "--reference-camera-resize-mode", type=str, default="CROP", choices=["CROP", "STRETCH", "LETTERBOX"], help="Resize mode for reference camera")
    parser.add_argument("-tcrm", "--target-camera-resize-mode", type=str, default="CROP", choices=["CROP", "STRETCH", "LETTERBOX"], help="Resize mode for target camera")

    args = parser.parse_args()
    resize_modes = {
        "CROP": dai.ImgResizeMode.CROP,
        "STRETCH": dai.ImgResizeMode.STRETCH,
        "LETTERBOX": dai.ImgResizeMode.LETTERBOX,
    }

    output_path = args.output_path
    duration = args.duration
    visualize = args.visualize
    rectified = args.rectified
    create_manip = args.manip
    fps = args.fps
    reference_camera_name = args.reference_camera
    target_camera_name = args.target_camera
    ref_camera_size = tuple(args.reference_camera_size)
    target_camera_size = tuple(args.target_camera_size)
    ref_camera_undistortion = args.reference_camera_undistortion
    target_camera_undistortion = args.target_camera_undistortion
    ref_camera_resize_mode = resize_modes[args.reference_camera_resize_mode]
    target_camera_resize_mode = resize_modes[args.target_camera_resize_mode]

    ref_metadata = create_writer_metadata(
        point_type="reference",
        socket=reference_camera_name,
        undistortion_enabled=ref_camera_undistortion,
        size=ref_camera_size,
        resize_mode=args.reference_camera_resize_mode,
        rectified=rectified,
    )
    
    target_metadata = create_writer_metadata(
        point_type="target",
        socket=target_camera_name,
        undistortion_enabled=target_camera_undistortion,
        size=target_camera_size,
        resize_mode=args.target_camera_resize_mode,
        rectified=rectified,
    )

    april_points_mcap_path = Path(f"{output_path}reference_APRIL_POINTS.mcap")
    ref_mcap_file, ref_mcap_writer, ref_mcap_channel_id = create_mcap_writer(april_points_mcap_path, topic=f"refrence_points", metadata=ref_metadata)
    target_april_points_mcap_path = Path(f"{output_path}_target_APRIL_POINTS.mcap")
    target_mcap_file, target_mcap_writer, target_mcap_channel_id = create_mcap_writer(target_april_points_mcap_path, topic=f"target_points", metadata=target_metadata)
    
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
            demux = pipeline.create(dai.node.MessageDemux)
            syncNode.out.link(demux.input)
            
            camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            cameras = {
                "CAM_A": camera_a,
                "CAM_B": camera_b,
                "CAM_C": camera_c,
            }

            # reference camera
            reference_camera = cameras[reference_camera_name]
            reference_camera_output = reference_camera.requestOutput(ref_camera_size,
                                                                     fps=fps,
                                                                     enableUndistortion=ref_camera_undistortion,
                                                                     resizeMode=ref_camera_resize_mode)
            reference_camera_output.link(syncNode.inputs["ref_camera"])
            link_record_video_node(pipeline, demux.outputs["ref_camera"], f"{output_path}{reference_camera_name}_reference", fps=fps)

            # reference camera AprilTag node
            april_tag_node = pipeline.create(dai.node.AprilTag)
            reference_camera_output.link(april_tag_node.inputImage)
            april_tag_node.out.link(syncNode.inputs["april_tags"])

            # depth

            if not rectified:
                camera_c_output = camera_c.requestOutput(
                    (640, 400),  fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
                )
                camera_b_output = camera_b.requestOutput(
                    (640, 400),  fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
                )
            else:
                camera_c_output = camera_c.requestOutput(
                    target_camera_size, fps=fps, enableUndistortion=target_camera_undistortion, resizeMode=target_camera_resize_mode)
                camera_b_output = camera_b.requestOutput(
                    target_camera_size, fps=fps, enableUndistortion=target_camera_undistortion, resizeMode=target_camera_resize_mode)

            stereo = pipeline.create(dai.node.StereoDepth)
            camera_b_output.link(stereo.left)
            camera_c_output.link(stereo.right)

            image_align = pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(image_align.input)
            reference_camera_output.link(image_align.inputAlignTo)
            image_align.outputAligned.link(syncNode.inputs["depth_aligned"])
            link_record_metadata_node(pipeline, demux.outputs["depth_aligned"], f"{output_path}_ALIGNED_DEPTH")

            # target camera
            target_camera = cameras[target_camera_name]

            if not rectified:
                target_camera_output = target_camera.requestOutput(
                    target_camera_size, fps=fps, enableUndistortion=target_camera_undistortion, resizeMode=target_camera_resize_mode
                )
            else:
                if target_camera_name == "CAM_B":
                    target_camera_output = stereo.rectifiedLeft
                else:
                    target_camera_output = stereo.rectifiedRight

            target_camera_output.link(syncNode.inputs["target_camera"])
            rectified_suffix = "_R" if rectified else ""
            link_record_video_node(pipeline, demux.outputs["target_camera"], f"{output_path}_{target_camera_name}{rectified_suffix}_target", fps=fps)

            target_april_tag_node = pipeline.create(dai.node.AprilTag) # if gray8 input (in the case of rectified image input) this will crash
            target_camera_output.link(target_april_tag_node.inputImage)
            target_april_tag_node.out.link(syncNode.inputs["target_april_tags"])

            sync_output_queue = syncNode.out.createOutputQueue(maxSize=1, blocking=True)


            print(f"Writing AprilTag points to: {april_points_mcap_path}")

            pipeline.start()
            recording_start_time = time.monotonic()
            seq_num = 0
            while pipeline.isRunning():
                assert sync_output_queue is not None
                message_group = sync_output_queue.get()
                assert isinstance(message_group, dai.MessageGroup)
                assert message_group.isSynced(int(1000000000 / fps * 0.5)), (
                    f"Sync node emitted unsynced group interval={message_group.getIntervalNs()}ns "
                    f"threshold={1000000000 / fps * 0.5}ns"
                )

                reference_frame = message_group["ref_camera"]
                target_frame = message_group["target_camera"]
                in_april_message = message_group["april_tags"]
                in_depth_message = message_group["depth_aligned"]
                in_target_april_message = message_group["target_april_tags"]

                assert isinstance(in_april_message, dai.AprilTags)
                assert isinstance(in_target_april_message, dai.AprilTags)
                assert isinstance(reference_frame, dai.ImgFrame)
                assert isinstance(target_frame, dai.ImgFrame)
                assert isinstance(in_depth_message, dai.ImgFrame)

                depth_frame = in_depth_message.getFrame()
                rgb_frame = ensure_bgr(reference_frame.getCvFrame())

                target_data_points = april_tag_to_mcap_points_list(in_target_april_message, None)
                reference_data_points = april_tag_to_mcap_points_list(in_april_message, depth_frame)

                write_apriltag_points(
                    writer=ref_mcap_writer,
                    channel_id=ref_mcap_channel_id,
                    ts=to_us(in_april_message.getTimestampDevice()),
                    data_points=reference_data_points,
                    seq_num=seq_num,
                )
                
                write_apriltag_points(
                    writer=target_mcap_writer,
                    channel_id=target_mcap_channel_id,
                    ts=to_us(in_target_april_message.getTimestampDevice()),
                    data_points=target_data_points,
                    seq_num=seq_num,
                )
                seq_num += 1

                overlay = draw_apriltag_points(
                    rgb_frame,
                    in_april_message,
                    spatial_points=reference_data_points,)
                cv2.imshow(f"save_data_points_{reference_camera_name}", overlay)
                
                target_overlay = draw_apriltag_points(
                    ensure_bgr(target_frame.getCvFrame()),
                    in_target_april_message,
                    spatial_points=target_data_points,
                )
                cv2.imshow(f"target_april_points_{target_camera_name}{rectified_suffix}", target_overlay)
                if cv2.waitKey(1) == ord("q"):
                    pipeline.stop()
                    break


                if seq_num > duration*fps:
                    pipeline.stop()
    finally:
        if visualize:
            cv2.destroyAllWindows()
        if ref_mcap_writer is not None:
            ref_mcap_writer.finish()
        if target_mcap_writer is not None:
            target_mcap_writer.finish()


if __name__ == "__main__":
    main()
