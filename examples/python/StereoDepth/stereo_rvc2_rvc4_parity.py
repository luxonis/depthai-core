#!/usr/bin/env python3

import argparse
import datetime
import json
import os
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import cv2
import depthai as dai
import numpy as np


@dataclass
class StereoEndpoint:
    pipeline: dai.Pipeline
    left: object
    right: object
    disparity: object
    rectified_left: object
    rectified_right: object


def parse_args():
    parser = argparse.ArgumentParser(description="Run byte-identical stereo frames on one RVC2 and one RVC4 device and compare their outputs")
    parser.add_argument("--left", type=Path, nargs="+")
    parser.add_argument("--right", type=Path, nargs="+")
    parser.add_argument("--rvc2-device", help="RVC2 device name or MXID; auto-selected when exactly one is available")
    parser.add_argument("--rvc2-firmware", type=Path, default=os.environ.get("DEPTHAI_RVC2_FIRMWARE"), help="RVC2 .mvcmd built for the RVC2 host schema")
    parser.add_argument("--rvc2-python", default=sys.executable, help="Python executable for the isolated RVC2 worker")
    parser.add_argument("--rvc2-pythonpath", default=os.environ.get("DEPTHAI_RVC2_PYTHONPATH"), help="PYTHONPATH containing a depthai binding matched to the RVC2 firmware")
    parser.add_argument("--rvc4-device", default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"), help="RVC4 IP, name, or device ID")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=800)
    parser.add_argument("--disparities", type=int, choices=(64, 96), default=96)
    parser.add_argument("--no-lr", action="store_true", help="disable left-right checking on both devices")
    parser.add_argument("--raw", action="store_true", help="disable decimation, median, speckle, spatial, and temporal filters on both devices")
    parser.add_argument("--decimation", type=int, choices=(1, 2, 3, 4), help="override the DEFAULT decimation factor on both devices")
    parser.add_argument("--speckle-range", type=int, help="override the speckle component size on both devices")
    parser.add_argument("--repeat", type=int, default=1, help="repeat the complete input sequence to exercise temporal state")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--max-mismatches", type=int, default=0, help="return success when at most this many pixels differ")
    parser.add_argument("--output", type=Path, help="save both outputs, absolute differences, and metrics.json")
    parser.add_argument("--worker-config", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker_config:
        return args
    if not args.left or not args.right:
        parser.error("--left and --right are required")
    if len(args.left) != len(args.right):
        parser.error("--left and --right must contain the same number of paths")
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    return args


def describe(info):
    return f"{info.name} ({info.deviceId}, {info.platform}, {info.state})"


def select_device(infos, platform, requested, label):
    candidates = [info for info in infos if info.platform == platform]
    if requested:
        candidates = [info for info in candidates if requested in (info.name, info.deviceId, info.getDeviceId())]
    if len(candidates) != 1:
        available = ", ".join(describe(info) for info in candidates) or "none"
        qualifier = f" matching {requested!r}" if requested else ""
        raise RuntimeError(f"Expected exactly one {label} device{qualifier}; candidates: {available}")
    return candidates[0]


def load_image(path):
    image = np.load(path) if path.suffix == ".npy" else cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    if image.dtype != np.uint8 or image.ndim != 2:
        raise ValueError(f"Expected an 8-bit grayscale image, got {image.dtype} {image.shape} from {path}")
    return image


def load_pair(left_path, right_path, size):
    left = load_image(left_path)
    right = load_image(right_path)
    if left.shape != right.shape:
        raise ValueError(f"Input dimensions differ: {left.shape} and {right.shape}")
    if left.shape[::-1] != size:
        left = cv2.resize(left, size, interpolation=cv2.INTER_AREA)
        right = cv2.resize(right, size, interpolation=cv2.INTER_AREA)
    return left, right


def load_pairs(args):
    source = [load_pair(Path(left), Path(right), (args.width, args.height)) for left, right in zip(args.left, args.right)]
    return [(f"{Path(left).stem}_{iteration}", pair) for iteration in range(args.repeat) for left, pair in zip(args.left, source)]


def frame(image, sequence, socket):
    message = dai.ImgFrame()
    message.setData(image.reshape(-1))
    message.setTimestamp(datetime.timedelta(milliseconds=sequence * 50))
    message.setSequenceNum(sequence)
    message.setInstanceNum(socket)
    message.setType(dai.ImgFrame.Type.RAW8)
    message.setWidth(image.shape[1])
    message.setStride(image.shape[1])
    message.setHeight(image.shape[0])
    return message


def configure_common(stereo, args):
    stereo.setInputResolution(args.width, args.height)
    stereo.setRectification(False)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    config = stereo.initialConfig
    if args.disparities == 64:
        config.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64
    if args.no_lr:
        config.setLeftRightCheck(False)
    if args.raw:
        config.postProcessing.decimationFilter.decimationFactor = 1
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.spatialFilter.enable = False
        config.postProcessing.temporalFilter.enable = False
    if args.decimation is not None:
        config.postProcessing.decimationFilter.decimationFactor = args.decimation
    if getattr(args, "speckle_range", None) is not None:
        config.postProcessing.speckleFilter.speckleRange = args.speckle_range


def build_endpoint(info, platform, args, passthrough=True):
    if platform == dai.Platform.RVC2 and args.rvc2_firmware:
        firmware = Path(args.rvc2_firmware)
        if not firmware.is_file():
            raise FileNotFoundError(firmware)
        device = dai.Device(info, firmware)
    else:
        device = dai.Device(info)
    pipeline = dai.Pipeline(device)
    if pipeline.getDefaultDevice().getPlatform() != platform:
        raise RuntimeError(f"Opened {describe(info)} as {pipeline.getDefaultDevice().getPlatform()}, expected {platform}")
    stereo = pipeline.create(dai.node.StereoDepth)
    if platform == dai.Platform.RVC2:
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    elif args.disparities == 64:
        stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT_64)
    else:
        stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT)
    configure_common(stereo, args)
    return StereoEndpoint(
        pipeline,
        stereo.left.createInputQueue(maxSize=4, blocking=True),
        stereo.right.createInputQueue(maxSize=4, blocking=True),
        stereo.disparity.createOutputQueue(maxSize=4, blocking=True),
        stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=True) if passthrough else None,
        stereo.rectifiedRight.createOutputQueue(maxSize=4, blocking=True) if passthrough else None,
    )


def receive(queue, sequence, deadline, label):
    while time.monotonic() < deadline:
        message = queue.tryGet()
        if message is None:
            time.sleep(0.005)
        elif message.getSequenceNum() == sequence:
            return message.getFrame().copy()
        elif message.getSequenceNum() > sequence:
            raise RuntimeError(f"{label} skipped sequence {sequence} and returned {message.getSequenceNum()}")
    raise TimeoutError(f"Timed out waiting for {label} sequence {sequence}")


def run_endpoint(endpoint, pairs, timeout, label):
    outputs = []
    with endpoint.pipeline:
        endpoint.pipeline.start()
        for sequence, (_, pair) in enumerate(pairs):
            endpoint.left.send(frame(pair[0], sequence, dai.CameraBoardSocket.CAM_B))
            endpoint.right.send(frame(pair[1], sequence, dai.CameraBoardSocket.CAM_C))
            deadline = time.monotonic() + timeout
            outputs.append(receive(endpoint.disparity, sequence, deadline, f"{label} disparity"))
            actual_left = receive(endpoint.rectified_left, sequence, deadline, f"{label} left passthrough")
            actual_right = receive(endpoint.rectified_right, sequence, deadline, f"{label} right passthrough")
            if not np.array_equal(actual_left, pair[0]) or not np.array_equal(actual_right, pair[1]):
                raise RuntimeError(f"{label} changed rectification-disabled input at sequence {sequence}")
    return outputs


def compare(sequence, name, rvc2, rvc4):
    if rvc2.shape != rvc4.shape or rvc2.dtype != rvc4.dtype:
        return {
            "sequence": sequence,
            "name": name,
            "rvc2_shape": rvc2.shape,
            "rvc4_shape": rvc4.shape,
            "rvc2_dtype": str(rvc2.dtype),
            "rvc4_dtype": str(rvc4.dtype),
            "mismatches": int(max(rvc2.size, rvc4.size)),
            "exact_percent": 0.0,
            "mae": None,
            "maximum_difference": None,
        }, None
    difference = rvc4.astype(np.int32) - rvc2.astype(np.int32)
    mismatch = difference != 0
    y, x = np.nonzero(mismatch)
    return {
        "sequence": sequence,
        "name": name,
        "shape": rvc2.shape,
        "dtype": str(rvc2.dtype),
        "rvc2_range": [int(rvc2.min()), int(rvc2.max())],
        "rvc4_range": [int(rvc4.min()), int(rvc4.max())],
        "mismatches": int(mismatch.sum()),
        "mismatch_bbox": [int(x.min()), int(y.min()), int(x.max()), int(y.max())] if x.size else None,
        "rvc2_zero_rvc4_nonzero": int(np.count_nonzero((rvc2 == 0) & (rvc4 != 0))),
        "rvc4_zero_rvc2_nonzero": int(np.count_nonzero((rvc4 == 0) & (rvc2 != 0))),
        "exact_percent": float(np.mean(~mismatch) * 100),
        "mae": float(np.mean(np.abs(difference))),
        "maximum_difference": int(np.max(np.abs(difference))),
    }, difference


def worker_args(config):
    return SimpleNamespace(
        left=[Path(path) for path in config["left"]],
        right=[Path(path) for path in config["right"]],
        rvc2_device=config["rvc2_device"],
        rvc2_firmware=Path(config["rvc2_firmware"]) if config.get("rvc2_firmware") else None,
        width=config["width"],
        height=config["height"],
        disparities=config["disparities"],
        no_lr=config["no_lr"],
        raw=config["raw"],
        decimation=config["decimation"],
        speckle_range=config.get("speckle_range"),
        repeat=config["repeat"],
        timeout=config["timeout"],
    )


def run_worker(config_path):
    config = json.loads(config_path.read_text())
    args = worker_args(config)
    pairs = load_pairs(args)
    infos = dai.Device.getAllAvailableDevices()
    info = select_device(infos, dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    endpoint = build_endpoint(info, dai.Platform.RVC2, args)
    outputs = run_endpoint(endpoint, pairs, args.timeout, "RVC2")
    np.savez(config["result"], **{f"frame_{index:03d}": output for index, output in enumerate(outputs)})
    print(f"RVC2 worker: {describe(info)}")
    print(f"RVC2 host: commit={getattr(dai, '__commit__', 'unknown')} embedded_device={getattr(dai, '__device_version__', 'unknown')}")


def launch_worker(args, rvc2_info, directory, pairs):
    config_path = directory / "worker.json"
    result_path = directory / "rvc2.npz"
    log_path = directory / "rvc2.log"
    left_paths = []
    right_paths = []
    for sequence, (_, pair) in enumerate(pairs):
        left_path = directory / f"{sequence:03d}_left.npy"
        right_path = directory / f"{sequence:03d}_right.npy"
        np.save(left_path, pair[0])
        np.save(right_path, pair[1])
        left_paths.append(str(left_path))
        right_paths.append(str(right_path))
    config = {
        "left": left_paths,
        "right": right_paths,
        "rvc2_device": rvc2_info.deviceId,
        "rvc2_firmware": str(args.rvc2_firmware) if args.rvc2_firmware else None,
        "width": args.width,
        "height": args.height,
        "disparities": args.disparities,
        "no_lr": args.no_lr,
        "raw": args.raw,
        "decimation": args.decimation,
        "speckle_range": args.speckle_range,
        "repeat": 1,
        "timeout": args.timeout,
        "result": str(result_path),
    }
    config_path.write_text(json.dumps(config))
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
    return process, result_path, log_path, log


def main():
    args = parse_args()
    if args.worker_config:
        run_worker(args.worker_config)
        return

    pairs = load_pairs(args)
    infos = dai.Device.getAllAvailableDevices()
    rvc2_info = select_device(infos, dai.XLinkPlatform.X_LINK_MYRIAD_X, args.rvc2_device, "RVC2")
    rvc4_info = select_device(infos, dai.XLinkPlatform.X_LINK_RVC4, args.rvc4_device, "RVC4")
    print(f"RVC2: {describe(rvc2_info)}")
    print(f"RVC4: {describe(rvc4_info)}")
    print(f"RVC4 host: commit={getattr(dai, '__commit__', 'unknown')} embedded_device={getattr(dai, '__device_rvc4_version__', 'unknown')}")
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="rvc2-rvc4-parity-") as temporary:
        worker, worker_result, worker_log_path, worker_log = launch_worker(args, rvc2_info, Path(temporary), pairs)
        try:
            rvc4 = build_endpoint(rvc4_info, dai.Platform.RVC4, args)
            rvc4_outputs = run_endpoint(rvc4, pairs, args.timeout, "RVC4")
            worker.wait(timeout=args.timeout * max(1, len(pairs)))
        except BaseException:
            if worker.poll() is None:
                worker.terminate()
                worker.wait()
            raise
        finally:
            worker_log.close()
        print(worker_log_path.read_text(), end="")
        if worker.returncode:
            raise RuntimeError(f"RVC2 worker exited with status {worker.returncode}")
        with np.load(worker_result) as archive:
            rvc2_outputs = [archive[f"frame_{index:03d}"] for index in range(len(pairs))]

    results = []
    for sequence, ((name, _), rvc2_disparity, rvc4_disparity) in enumerate(zip(pairs, rvc2_outputs, rvc4_outputs)):
        metrics, difference = compare(sequence, name, rvc2_disparity, rvc4_disparity)
        results.append(metrics)
        print(json.dumps(metrics, sort_keys=True))
        if args.output:
            cv2.imwrite(str(args.output / f"{sequence:03d}_{name}_rvc2.png"), rvc2_disparity)
            cv2.imwrite(str(args.output / f"{sequence:03d}_{name}_rvc4.png"), rvc4_disparity)
            if difference is not None:
                cv2.imwrite(str(args.output / f"{sequence:03d}_{name}_absdiff.png"), np.abs(difference).astype(np.uint16))

    total_pixels = sum(int(np.prod(result.get("shape", result.get("rvc2_shape")))) for result in results)
    total_mismatches = sum(result["mismatches"] for result in results)
    summary = {
        "frames": len(results),
        "pixels": total_pixels,
        "disparities": args.disparities,
        "lr": not args.no_lr,
        "raw": args.raw,
        "decimation": args.decimation if args.decimation is not None else (1 if args.raw else 2),
        "speckle_range": args.speckle_range,
        "mismatches": total_mismatches,
        "exact_percent": float((total_pixels - total_mismatches) * 100 / total_pixels),
        "passed": total_mismatches <= args.max_mismatches,
    }
    print(json.dumps({"summary": summary}, sort_keys=True))
    if args.output:
        (args.output / "metrics.json").write_text(json.dumps({"frames": results, "summary": summary}, indent=2))
    if not summary["passed"]:
        raise RuntimeError(f"RVC2/RVC4 parity failed: {total_mismatches} mismatches exceed {args.max_mismatches}")


if __name__ == "__main__":
    main()
