#!/usr/bin/env python3

import argparse
import datetime
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import depthai as dai
import numpy as np

import stereo_rvc2_rvc4_parity as parity


LENGTH = struct.Struct("!I")


def parse_args():
    parser = argparse.ArgumentParser(description="Feed live synchronized RVC2 stereo frames through RVC4 and compare both disparity outputs")
    parser.add_argument("--rvc2-device", help="RVC2 device name or MXID; auto-selected when exactly one is available")
    parser.add_argument("--rvc2-firmware", type=Path, default=os.environ.get("DEPTHAI_RVC2_FIRMWARE"), help="RVC2 .mvcmd matched to the RVC2 Python binding")
    parser.add_argument("--rvc2-python", default=sys.executable, help="Python executable for the isolated RVC2 worker")
    parser.add_argument("--rvc2-pythonpath", default=os.environ.get("DEPTHAI_RVC2_PYTHONPATH"), help="PYTHONPATH containing a depthai binding matched to the RVC2 firmware")
    parser.add_argument("--rvc4-device", default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"), help="RVC4 IP, name, or device ID")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--frames", type=int, default=100, help="number of live frames included in the parity result")
    parser.add_argument("--warmup-frames", type=int, default=10, help="number of startup frames processed but excluded from the parity result")
    parser.add_argument("--disparities", type=int, choices=(64, 96), default=96)
    parser.add_argument("--no-lr", action="store_true")
    parser.add_argument("--raw", action="store_true")
    parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4))
    parser.add_argument("--max-mismatches", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--verify-rvc4-passthrough", action="store_true", help="read back and compare both full-resolution RVC4 inputs on every frame")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--save-frames", action="store_true", help="save every synchronized input and disparity frame for deterministic replay")
    parser.add_argument("--output", type=Path, help="save metrics and the latest live comparison image")
    parser.add_argument("--worker-config", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker_config:
        return args
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.warmup_frames < 0:
        parser.error("--warmup-frames cannot be negative")
    if args.fps <= 0:
        parser.error("--fps must be positive")
    return args


def receive_exact(connection, size):
    result = bytearray(size)
    view = memoryview(result)
    received = 0
    while received < size:
        count = connection.recv_into(view[received:])
        if count == 0:
            raise EOFError("RVC2 bridge closed")
        received += count
    return result


def send_packet(connection, sequence, left_timestamp, right_timestamp, left, right, disparity):
    arrays = {"left": left, "right": right, "disparity": disparity}
    metadata = {
        "sequence": sequence,
        "left_timestamp_us": round(left_timestamp.total_seconds() * 1_000_000),
        "right_timestamp_us": round(right_timestamp.total_seconds() * 1_000_000),
        "arrays": {name: {"shape": value.shape, "dtype": str(value.dtype), "bytes": value.nbytes} for name, value in arrays.items()},
    }
    encoded = json.dumps(metadata).encode()
    connection.sendall(LENGTH.pack(len(encoded)))
    connection.sendall(encoded)
    for value in arrays.values():
        connection.sendall(value.tobytes())


def receive_packet(connection):
    metadata_size = LENGTH.unpack(receive_exact(connection, LENGTH.size))[0]
    metadata = json.loads(receive_exact(connection, metadata_size))
    arrays = {}
    for name, description in metadata["arrays"].items():
        payload = receive_exact(connection, description["bytes"])
        arrays[name] = np.frombuffer(payload, dtype=np.dtype(description["dtype"])).reshape(description["shape"]).copy()
    return metadata, arrays


def worker_args(config):
    return SimpleNamespace(
        rvc2_device=config["rvc2_device"],
        rvc2_firmware=Path(config["rvc2_firmware"]) if config.get("rvc2_firmware") else None,
        width=config["width"],
        height=config["height"],
        fps=config["fps"],
        frames=config["frames"],
        disparities=config["disparities"],
        no_lr=config["no_lr"],
        raw=config["raw"],
        decimation=config["decimation"],
        timeout=config["timeout"],
        socket=Path(config["socket"]),
    )


def receive_sequence(queue, sequence, deadline, label):
    while time.monotonic() < deadline:
        message = queue.tryGet()
        if message is None:
            time.sleep(0.002)
        elif message.getSequenceNum() == sequence:
            return message
        elif message.getSequenceNum() > sequence:
            raise RuntimeError(f"{label} skipped sequence {sequence} and returned {message.getSequenceNum()}")
    raise TimeoutError(f"Timed out waiting for {label} sequence {sequence}")


def run_worker(config_path):
    config = json.loads(config_path.read_text())
    args = worker_args(config)
    infos = dai.Device.getAllAvailableDevices()
    info = parity.select_device(infos, dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    if args.rvc2_firmware:
        device = dai.Device(info, args.rvc2_firmware)
    else:
        device = dai.Device(info)
    pipeline = dai.Pipeline(device)
    pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
    left_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=args.fps)
    right_camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=args.fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    parity.configure_common(stereo, args)
    left_camera.requestOutput((args.width, args.height), fps=args.fps).link(stereo.left)
    right_camera.requestOutput((args.width, args.height), fps=args.fps).link(stereo.right)
    left_queue = stereo.syncedLeft.createOutputQueue(maxSize=8, blocking=True)
    right_queue = stereo.syncedRight.createOutputQueue(maxSize=8, blocking=True)
    disparity_queue = stereo.disparity.createOutputQueue(maxSize=8, blocking=True)

    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.connect(str(args.socket))
        with pipeline:
            pipeline.start()
            for _ in range(args.frames):
                disparity_message = disparity_queue.get()
                sequence = disparity_message.getSequenceNum()
                deadline = time.monotonic() + args.timeout
                left_message = receive_sequence(left_queue, sequence, deadline, "RVC2 synced left")
                right_message = receive_sequence(right_queue, sequence, deadline, "RVC2 synced right")
                send_packet(
                    connection,
                    sequence,
                    left_message.getTimestamp(),
                    right_message.getTimestamp(),
                    left_message.getFrame(),
                    right_message.getFrame(),
                    disparity_message.getFrame(),
                )
    print(f"RVC2 worker: {parity.describe(info)}")
    print(f"RVC2 host: commit={getattr(dai, '__commit__', 'unknown')} embedded_device={getattr(dai, '__device_version__', 'unknown')}")


def launch_worker(args, info, directory):
    socket_path = directory / "live.sock"
    config_path = directory / "worker.json"
    log_path = directory / "rvc2.log"
    config = {
        "rvc2_device": info.deviceId,
        "rvc2_firmware": str(args.rvc2_firmware) if args.rvc2_firmware else None,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "frames": args.frames + args.warmup_frames,
        "disparities": args.disparities,
        "no_lr": args.no_lr,
        "raw": args.raw,
        "decimation": args.decimation,
        "timeout": args.timeout,
        "socket": str(socket_path),
    }
    config_path.write_text(json.dumps(config))
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(1)
    server.settimeout(args.timeout)
    environment = os.environ.copy()
    if args.rvc2_pythonpath:
        environment["PYTHONPATH"] = args.rvc2_pythonpath
    log = log_path.open("w")
    process = subprocess.Popen(
        [args.rvc2_python, str(Path(__file__).resolve()), "--worker-config", str(config_path)],
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return process, server, log, log_path


def input_frame(image, sequence, socket_name, timestamp_us):
    message = parity.frame(image, sequence, socket_name)
    message.setTimestamp(datetime.timedelta(microseconds=timestamp_us))
    return message


def colorize(disparity, maximum):
    normalized = np.clip(disparity.astype(np.float32) * (255.0 / maximum), 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[disparity == 0] = 0
    return colored


def make_visualization(left, rvc2, rvc4, difference, metrics, bridge_fps, maximum):
    output_size = (rvc2.shape[1], rvc2.shape[0])
    left_view = cv2.cvtColor(cv2.resize(left, output_size, interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
    rvc2_view = colorize(rvc2, maximum)
    rvc4_view = colorize(rvc4, maximum)
    absolute = np.abs(difference)
    difference_view = cv2.applyColorMap(np.clip(absolute, 0, 255).astype(np.uint8), cv2.COLORMAP_HOT)
    difference_view[absolute == 0] = 0
    views = ((left_view, "RVC2 live left"), (rvc2_view, "RVC2 DEFAULT"), (rvc4_view, "RVC4 DEFAULT"), (difference_view, "absolute difference"))
    labeled = []
    for view, title in views:
        current = view.copy()
        cv2.putText(current, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        labeled.append(current)
    mosaic = np.vstack((np.hstack(labeled[:2]), np.hstack(labeled[2:])))
    text = f"exact {metrics['exact_percent']:.8f}% | mismatches {metrics['mismatches']} | bridge {bridge_fps:.2f} FPS"
    cv2.putText(mosaic, text, (12, mosaic.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return mosaic


def main():
    args = parse_args()
    if args.worker_config:
        run_worker(args.worker_config)
        return

    infos = dai.Device.getAllAvailableDevices()
    rvc2_info = parity.select_device(infos, dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    rvc4_info = parity.select_device(infos, dai.XLinkPlatform.X_LINK_RVC4, args.rvc4_device, "RVC4")
    print(f"RVC2: {parity.describe(rvc2_info)}")
    print(f"RVC4: {parity.describe(rvc4_info)}")
    print(f"RVC4 host: commit={getattr(dai, '__commit__', 'unknown')} embedded_device={getattr(dai, '__device_rvc4_version__', 'unknown')}")
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        if args.save_frames:
            (args.output / "frames").mkdir(exist_ok=True)

    results = []
    total_pixels = 0
    total_mismatches = 0
    worker_log_text = ""
    worker_returncode = None
    latest_visualization = None
    first_source_timestamp_us = None
    with tempfile.TemporaryDirectory(prefix="rvc2-rvc4-live-") as temporary:
        worker, server, worker_log, worker_log_path = launch_worker(args, rvc2_info, Path(temporary))
        stopped_early = False
        try:
            endpoint = parity.build_endpoint(rvc4_info, dai.Platform.RVC4, args, passthrough=args.verify_rvc4_passthrough)
            with endpoint.pipeline:
                endpoint.pipeline.start()
                connection, _ = server.accept()
                bridge_started = time.monotonic()
                with connection:
                    for index in range(args.warmup_frames + args.frames):
                        metadata, arrays = receive_packet(connection)
                        source_sequence = metadata["sequence"]
                        if first_source_timestamp_us is None:
                            first_source_timestamp_us = metadata["left_timestamp_us"]
                        left = arrays["left"]
                        right = arrays["right"]
                        rvc2_disparity = arrays["disparity"]
                        endpoint.left.send(input_frame(left, source_sequence, dai.CameraBoardSocket.CAM_B, metadata["left_timestamp_us"]))
                        endpoint.right.send(input_frame(right, source_sequence, dai.CameraBoardSocket.CAM_C, metadata["right_timestamp_us"]))
                        deadline = time.monotonic() + args.timeout
                        rvc4_disparity = parity.receive(endpoint.disparity, source_sequence, deadline, "RVC4 disparity")
                        if args.verify_rvc4_passthrough:
                            rvc4_left = parity.receive(endpoint.rectified_left, source_sequence, deadline, "RVC4 left passthrough")
                            rvc4_right = parity.receive(endpoint.rectified_right, source_sequence, deadline, "RVC4 right passthrough")
                            if not np.array_equal(left, rvc4_left) or not np.array_equal(right, rvc4_right):
                                raise RuntimeError(f"RVC4 changed live input sequence {source_sequence}")
                        metrics, difference = parity.compare(source_sequence, f"live_{source_sequence}", rvc2_disparity, rvc4_disparity)
                        now = time.monotonic()
                        bridge_fps = (index + 1) / max(now - bridge_started, 1e-9)
                        metrics["bridge_fps"] = bridge_fps
                        metrics["warmup"] = index < args.warmup_frames
                        if not metrics["warmup"]:
                            results.append(metrics)
                            total_pixels += rvc2_disparity.size
                            total_mismatches += metrics["mismatches"]
                        if args.output and args.save_frames:
                            prefix = args.output / "frames" / f"{index:03d}_{source_sequence}"
                            np.save(f"{prefix}_left.npy", left)
                            np.save(f"{prefix}_right.npy", right)
                            np.save(f"{prefix}_rvc2.npy", rvc2_disparity)
                            np.save(f"{prefix}_rvc4.npy", rvc4_disparity)
                        print(json.dumps(metrics, sort_keys=True))
                        make_view = not args.no_display or (args.output and index + 1 == args.warmup_frames + args.frames)
                        if make_view:
                            maximum = 7600 if args.disparities == 96 else 8064
                            visualization = make_visualization(left, rvc2_disparity, rvc4_disparity, difference, metrics, bridge_fps, maximum)
                            if not metrics["warmup"]:
                                latest_visualization = visualization
                            if not args.no_display:
                                cv2.imshow("Live RVC2 to RVC4 parity", visualization)
                                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                                    stopped_early = True
                                    break
            if stopped_early:
                worker.terminate()
            worker.wait(timeout=args.timeout)
            worker_returncode = worker.returncode
        except BaseException:
            if worker.poll() is None:
                worker.terminate()
                worker.wait()
            raise
        finally:
            server.close()
            worker_log.close()
            if worker_log_path.exists():
                worker_log_text = worker_log_path.read_text()
            cv2.destroyAllWindows()
    print(worker_log_text, end="")
    if worker_returncode and not stopped_early:
        raise RuntimeError(f"RVC2 worker exited with status {worker_returncode}")
    if total_pixels == 0:
        raise RuntimeError("No post-warmup frames were compared")

    exact_percent = (total_pixels - total_mismatches) * 100 / total_pixels
    source_duration = (metadata["left_timestamp_us"] - first_source_timestamp_us) / 1_000_000
    summary = {
        "frames": len(results),
        "pixels": total_pixels,
        "mismatches": total_mismatches,
        "exact_percent": exact_percent,
        "source_fps": (args.warmup_frames + len(results) - 1) / source_duration if source_duration > 0 else None,
        "bridge_fps": metrics["bridge_fps"],
        "rvc4_passthrough_verified": args.verify_rvc4_passthrough,
        "passed": total_mismatches <= args.max_mismatches,
    }
    print(json.dumps({"summary": summary}, sort_keys=True))
    if args.output:
        (args.output / "metrics.json").write_text(json.dumps({"frames": results, "summary": summary}, indent=2))
        if latest_visualization is not None:
            cv2.imwrite(str(args.output / "latest.png"), latest_visualization)
    if not summary["passed"]:
        raise RuntimeError(f"Live RVC2/RVC4 parity failed: {total_mismatches} mismatches exceed {args.max_mismatches}")


if __name__ == "__main__":
    main()
