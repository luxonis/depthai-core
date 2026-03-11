#!/usr/bin/env python3

import argparse
import json
import os
import time
from datetime import timedelta
from pathlib import Path
from typing import Any

import cv2
import depthai as dai


STREAM_ORDER = ["CAM_A", "CAM_D", "CAM_E", "CAM_B", "CAM_C", "CAM_B_R", "CAM_C_R"]


def parse_size(value: str) -> tuple[int, int]:
    normalized = value.strip().lower().replace("x", ",")
    parts = [part.strip() for part in normalized.split(",") if part.strip()]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("size must be in WIDTHxHEIGHT format (for example 1280x720)")
    try:
        width = int(parts[0])
        height = int(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError("size values must be integers") from exc
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("size values must be positive")
    return width, height


def to_ns(ts: Any) -> int:
    if ts is None:
        return 0
    if isinstance(ts, int):
        return ts
    if isinstance(ts, float):
        return int(ts * 1_000_000_000)
    if hasattr(ts, "total_seconds"):
        return int(ts.total_seconds() * 1_000_000_000)
    raise TypeError(f"Unsupported timestamp type: {type(ts)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Save a single synchronized capture from the same 7 streams as save_data_points.py. "
            "Press 'c' in the preview window to capture once."
        )
    )
    parser.add_argument(
        "-o",
        "--output-path",
        default="capture/single_frame",
        help="Output file prefix. Files are saved as <prefix>_<STREAM>.png and <prefix>_meta.json.",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Camera FPS")
    parser.add_argument("--size", type=parse_size, default=(1280, 720), help="Resolution for CAM_E as WIDTHxHEIGHT")
    parser.add_argument(
        "--sync-threshold-ms",
        type=float,
        default=None,
        help="Sync threshold in milliseconds. Defaults to half frame period.",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=10,
        help="Skip this many synced groups before allowing capture.",
    )
    return parser.parse_args()


def stream_path(prefix: Path, stream_name: str) -> Path:
    return prefix.parent / f"{prefix.name}_{stream_name}.png"


def metadata_path(prefix: Path) -> Path:
    return prefix.parent / f"{prefix.name}_meta.json"


def calibration_path(prefix: Path) -> Path:
    return prefix.parent / f"{prefix.name}_calibration.json"


def save_group(
    message_group: dai.MessageGroup,
    prefix: Path,
    sync_threshold_ns: int,
) -> None:
    meta: dict[str, Any] = {
        "saved_at_epoch_ns": time.time_ns(),
        "sync_threshold_ns": sync_threshold_ns,
        "group_interval_ns": int(message_group.getIntervalNs()),
        "streams": {},
    }

    for stream in STREAM_ORDER:
        msg = message_group[stream]
        if not isinstance(msg, dai.ImgFrame):
            raise RuntimeError(f"Expected ImgFrame for {stream}, got {type(msg)}")

        frame = msg.getCvFrame()
        out_path = stream_path(prefix, stream)
        ok = cv2.imwrite(str(out_path), frame)
        if not ok:
            raise RuntimeError(f"Failed to save image for {stream} to {out_path}")

        meta["streams"][stream] = {
            "file": out_path.name,
            "sequence_num": int(msg.getSequenceNum()),
            "timestamp_host_ns": to_ns(msg.getTimestamp()),
            "timestamp_device_ns": to_ns(msg.getTimestampDevice()),
            "width": int(msg.getWidth()),
            "height": int(msg.getHeight()),
        }

    with metadata_path(prefix).open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, sort_keys=True)


def main() -> None:
    args = parse_args()
    output_prefix = Path(args.output_path)
    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    fps = float(args.fps)
    if fps <= 0:
        raise ValueError("fps must be positive")

    sync_threshold_ns = int((1_000_000_000.0 / fps) * 0.5)
    if args.sync_threshold_ms is not None:
        sync_threshold_ns = int(args.sync_threshold_ms * 1_000_000.0)
    if sync_threshold_ns <= 0:
        raise ValueError("sync threshold must be positive")

    with dai.Pipeline() as pipeline:
        camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        cam_a = camera_a.requestOutput((1280, 720), type=dai.ImgFrame.Type.NV12, fps=fps, enableUndistortion=True)
        cam_d = camera_a.requestOutput(
            (1080, 400),
            fps=fps,
            resizeMode=dai.ImgResizeMode.STRETCH,
            enableUndistortion=False,
        )
        cam_e = camera_a.requestOutput(
            args.size,
            type=dai.ImgFrame.Type.NV12,
            fps=fps,
            resizeMode=dai.ImgResizeMode.LETTERBOX,
            enableUndistortion=True,
        )

        camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        cam_b = camera_b.requestFullResolutionOutput(fps=fps)
        cam_c = camera_c.requestFullResolutionOutput(fps=fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
        cam_b.link(stereo.left)
        cam_c.link(stereo.right)

        stream_outputs: dict[str, dai.Node.Output] = {
            "CAM_A": cam_a,
            "CAM_D": cam_d,
            "CAM_E": cam_e,
            "CAM_B": cam_b,
            "CAM_C": cam_c,
            "CAM_B_R": stereo.rectifiedLeft,
            "CAM_C_R": stereo.rectifiedRight,
        }

        sync_node = pipeline.create(dai.node.Sync)
        sync_node.setRunOnHost(True)
        sync_node.setSyncThreshold(timedelta(microseconds=sync_threshold_ns / 1_000.0))
        sync_node.setSyncAttempts(-1)
        for stream, output in stream_outputs.items():
            output.link(sync_node.inputs[stream])

        sync_queue = sync_node.out.createOutputQueue(maxSize=1, blocking=True)

        print("Starting pipeline...")
        pipeline.start()

        calib_file = calibration_path(output_prefix)
        pipeline.getDefaultDevice().readCalibration().eepromToJsonFile(str(calib_file))
        print(f"Saved calibration to: {calib_file}")
        print(f"Capture sync threshold: {sync_threshold_ns / 1_000_000.0:.3f} ms")

        display_available = os.environ.get("DISPLAY", "") != ""
        capture_requested = False
        warmup_seen = 0
        stop_requested = False

        if display_available:
            print("Preview ready. Press 'c' to capture one synced frame set, or 'q' to quit.")
        else:
            print("DISPLAY not set. Press ENTER to capture one synced frame set, or Ctrl+C to quit.")
            try:
                input()
            except KeyboardInterrupt:
                stop_requested = True
            capture_requested = not stop_requested

        while pipeline.isRunning() and not stop_requested:
            message_group = sync_queue.tryGet()
            if message_group is None:
                if display_available:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        stop_requested = True
                    elif key == ord("c"):
                        capture_requested = True
                time.sleep(0.001)
                continue

            if not isinstance(message_group, dai.MessageGroup):
                continue

            if display_available:
                preview_msg = message_group["CAM_A"]
                if isinstance(preview_msg, dai.ImgFrame):
                    preview = preview_msg.getCvFrame().copy()
                    cv2.putText(
                        preview,
                        "Press 'c' to capture, 'q' to quit",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow("save_single_frame", preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_requested = True
                    continue
                if key == ord("c"):
                    capture_requested = True

            warmup_seen += 1
            if warmup_seen <= args.warmup_frames:
                continue
            if not capture_requested:
                continue

            if not message_group.isSynced(sync_threshold_ns):
                print(
                    f"Skipping unsynced group: interval={message_group.getIntervalNs()} ns, "
                    f"threshold={sync_threshold_ns} ns"
                )
                continue

            save_group(message_group=message_group, prefix=output_prefix, sync_threshold_ns=sync_threshold_ns)
            print(f"Saved single synced capture set with prefix: {output_prefix}")
            break

        if pipeline.isRunning():
            pipeline.stop()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
