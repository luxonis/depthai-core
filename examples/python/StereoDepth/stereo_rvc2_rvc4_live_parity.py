#!/usr/bin/env python3
"""Compare RVC4's RVC2 stereo backend with an actual RVC2, bit for bit.

The script captures rectified mono pairs once, then replays those identical
inputs through both devices. Disparity and depth can be checked in the same
replay. Any output mismatch, missing frame, unsupported configuration, or
requested performance failure produces a nonzero exit code.
"""

import argparse
import dataclasses
import datetime
import hashlib
import json
import os
import time
from pathlib import Path

import depthai as dai
import numpy as np


SIZE = (1280, 800)
PRESETS = {
    name.lower().replace("_", "-"): getattr(dai.node.StereoDepth.PresetMode, name)
    for name in ("FAST_ACCURACY", "FAST_DENSITY", "DEFAULT", "FACE", "HIGH_DETAIL", "ROBOTICS", "DENSITY", "ACCURACY")
}
FILTER_CASES = (
    "native",
    "raw",
    "decimation-off",
    "median-off",
    "speckle-off",
    "spatial-off",
    "temporal-off",
    "threshold-off",
    "brightness-limited",
    "brightness-max-255",
    "hole-filling-off",
    "adaptive-median-off",
)
OUTPUTS = ("disparity", "depth")
OUTPUT_DTYPES = {"disparity": ("uint8", "uint16"), "depth": ("uint16",)}
CONCLUSIVE_FILTER_CASES = {
    "raw",
    "decimation-off",
    "median-off",
    "speckle-off",
    "spatial-off",
    "temporal-off",
    "threshold-off",
    "brightness-limited",
}
INERT_FILTER_CASES = {"hole-filling-off", "adaptive-median-off"}


@dataclasses.dataclass(frozen=True)
class ReplayPair:
    """One deterministic stereo replay input with its original metadata."""

    left: np.ndarray
    right: np.ndarray
    left_transformation: dai.ImgTransformation
    right_transformation: dai.ImgTransformation
    timing: dict


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rvc2-device", help="RVC2 name or MXID")
    parser.add_argument(
        "--rvc4-device",
        default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST"),
    )
    parser.add_argument("--preset", action="append", choices=(*PRESETS, "all"), default=[])
    parser.add_argument("--width", action="append", type=int, choices=(64, 96), default=[])
    parser.add_argument("--filter-case", action="append", choices=FILTER_CASES, default=[])
    parser.add_argument(
        "--filter-preset",
        action="append",
        choices=(*PRESETS, "all"),
        default=[],
        help="presets that receive non-native filter cases (default: every selected preset)",
    )
    parser.add_argument(
        "--output",
        action="append",
        choices=OUTPUTS,
        default=[],
        help="numeric output to compare; repeat to compare both in one replay (default: disparity)",
    )
    parser.add_argument("--frames", type=int, default=30, help="measured frames per configuration")
    parser.add_argument("--warmup", type=int, default=5, help="unmeasured frames per configuration")
    parser.add_argument("--capture-fps", type=float, default=10.0)
    parser.add_argument("--in-flight", type=int, default=4, help="replay pairs kept in flight to exercise DSP/GPU overlap")
    parser.add_argument(
        "--runtime-config",
        action="store_true",
        help="apply non-native filter cases through StereoDepth.inputConfig",
    )
    parser.add_argument("--alpha", type=float, default=-5.0, help="RVC2 rectification alpha")
    parser.add_argument("--min-rvc4-fps", type=float, default=0.0)
    parser.add_argument("--min-rvc4-rvc2-ratio", type=float, default=0.0)
    parser.add_argument(
        "--require-conclusive-filters",
        action="store_true",
        help="require output-changing ablations to differ and RVC2-inert controls to remain identical",
    )
    parser.add_argument("--corpus", type=Path, help="persist rectified inputs and actual-RVC2 output oracles")
    parser.add_argument("--rvc4-only", action="store_true", help="compare only RVC4 against an existing --corpus")
    parser.add_argument(
        "--oracle-only", action="store_true", help="create an actual-RVC2 --corpus without running RVC4"
    )
    args = parser.parse_args()
    if args.frames < 2 or args.warmup < 0 or args.capture_fps <= 0 or args.in_flight < 1:
        parser.error("--frames must be at least 2; --capture-fps and --in-flight must be positive; --warmup must not be negative")
    if args.rvc4_only and args.corpus is None:
        parser.error("--rvc4-only requires --corpus")
    if args.oracle_only and args.corpus is None:
        parser.error("--oracle-only requires --corpus")
    if args.rvc4_only and args.oracle_only:
        parser.error("--rvc4-only and --oracle-only are mutually exclusive")
    if not args.rvc4_only and args.rvc2_device is None:
        parser.error("--rvc2-device is required unless --rvc4-only is used")
    if not args.oracle_only and args.rvc4_device is None:
        parser.error("--rvc4-device is required unless --oracle-only is used")
    args.preset = list(PRESETS) if "all" in args.preset else (args.preset or ["default"])
    args.width = args.width or [96]
    args.filter_case = args.filter_case or ["native"]
    args.filter_preset = (
        list(PRESETS)
        if "all" in args.filter_preset
        else (args.filter_preset or list(args.preset))
    )
    args.output = list(dict.fromkeys(args.output or ["disparity"]))
    return args


def find_device(identifier, platform):
    matches = [
        info
        for info in dai.Device.getAllAvailableDevices()
        if info.platform == platform and identifier in (info.name, info.deviceId, info.getDeviceId())
    ]
    if len(matches) != 1:
        found = ", ".join(f"{info.name} ({info.deviceId})" for info in matches) or "none"
        raise RuntimeError(f"Expected one {platform.name} device matching {identifier!r}; found {found}")
    return matches[0]


def device_record(info):
    return {"name": info.name, "device_id": info.deviceId, "platform": info.platform.name}


def array_sha256(array):
    value = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(value.dtype.str.encode())
    digest.update(np.asarray(value.shape, dtype=np.uint64).tobytes())
    digest.update(value.tobytes())
    return digest.hexdigest()


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def runtime_record():
    record = {
        "depthai_version": getattr(dai, "__version__", "unknown"),
        "depthai_commit": getattr(dai, "__commit__", "unknown"),
        "rvc4_resource_version": getattr(dai, "__device_rvc4_version__", "unknown"),
        "rvc2_resource_version": getattr(dai, "__device_version__", "unknown"),
    }
    fwp_value = os.environ.get("DEPTHAI_DEVICE_RVC4_FWP")
    if fwp_value:
        fwp = Path(fwp_value).resolve()
        record["rvc4_fwp"] = {
            "path": str(fwp),
            "sha256": file_sha256(fwp),
        }
    rvc2_binary_value = os.environ.get("DEPTHAI_DEVICE_BINARY")
    if rvc2_binary_value:
        binary = Path(rvc2_binary_value).resolve()
        record["rvc2_binary"] = {
            "path": str(binary),
            "sha256": file_sha256(binary),
        }
    return record


def point3_record(point):
    return [point.x, point.y, point.z]


def transformation_record(transformation):
    extrinsics = transformation.getExtrinsics()
    return {
        "source_size": list(transformation.getSourceSize()),
        "size": list(transformation.getSize()),
        "matrix": transformation.getMatrix(),
        "source_intrinsics": transformation.getSourceIntrinsicMatrix(),
        "distortion_model": transformation.getDistortionModel().name,
        "distortion_coefficients": transformation.getDistortionCoefficients(),
        "extrinsics": {
            "rotation_matrix": extrinsics.rotationMatrix,
            "translation": point3_record(extrinsics.translation),
            "spec_translation": point3_record(extrinsics.specTranslation),
            "to_camera_socket": extrinsics.toCameraSocket.name,
            "length_unit": extrinsics.lengthUnit.name,
        },
        "source_crops": [
            {
                "center": [crop.center.x, crop.center.y],
                "size": [crop.size.width, crop.size.height],
                "angle": crop.angle,
                "normalized": crop.isNormalized(),
            }
            for crop in transformation.getSrcCrops()
        ],
    }


def transformation_from_record(record):
    extrinsics_record = record["extrinsics"]
    extrinsics = dai.Extrinsics()
    extrinsics.rotationMatrix = extrinsics_record["rotation_matrix"]
    extrinsics.translation = dai.Point3f(*extrinsics_record["translation"])
    extrinsics.specTranslation = dai.Point3f(*extrinsics_record["spec_translation"])
    extrinsics.toCameraSocket = getattr(dai.CameraBoardSocket, extrinsics_record["to_camera_socket"])
    extrinsics.lengthUnit = getattr(dai.LengthUnit, extrinsics_record["length_unit"])

    source_width, source_height = record["source_size"]
    transformation = dai.ImgTransformation(
        source_width,
        source_height,
        record["source_intrinsics"],
        getattr(dai.CameraModel, record["distortion_model"]),
        record["distortion_coefficients"],
        extrinsics,
    )
    transformation.setSize(*record["size"])
    transformation.addTransformation(record["matrix"])
    transformation.addSrcCrops(
        [
            dai.RotatedRect(
                dai.Point2f(*crop["center"], crop["normalized"]),
                dai.Size2f(*crop["size"], crop["normalized"]),
                crop["angle"],
            )
            for crop in record["source_crops"]
        ]
    )
    return transformation


def checked_transformation_record(transformation):
    record = transformation_record(transformation)
    reconstructed = transformation_from_record(record)
    if not transformation.isEqualTransformation(reconstructed) or transformation_record(reconstructed) != record:
        raise RuntimeError("ImgTransformation metadata cannot be reconstructed exactly")
    return record


def configure_case(config, case):
    if case == "native":
        return
    if case == "raw":
        config.postProcessing.decimationFilter.decimationFactor = 1
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.spatialFilter.enable = False
        config.postProcessing.temporalFilter.enable = False
        config.postProcessing.holeFilling.enable = False
        config.postProcessing.adaptiveMedianFilter.enable = False
        config.postProcessing.thresholdFilter.minRange = 0
        config.postProcessing.thresholdFilter.maxRange = 65535
        config.postProcessing.brightnessFilter.minBrightness = -1
        config.postProcessing.brightnessFilter.maxBrightness = 256
    elif case == "decimation-off":
        config.postProcessing.decimationFilter.decimationFactor = 1
    elif case == "median-off":
        config.setMedianFilter(dai.MedianFilter.MEDIAN_OFF)
    elif case == "speckle-off":
        config.postProcessing.speckleFilter.enable = False
    elif case == "spatial-off":
        config.postProcessing.spatialFilter.enable = False
    elif case == "temporal-off":
        config.postProcessing.temporalFilter.enable = False
    elif case == "threshold-off":
        config.postProcessing.thresholdFilter.minRange = 0
        config.postProcessing.thresholdFilter.maxRange = 65535
    elif case == "brightness-limited":
        config.postProcessing.brightnessFilter.minBrightness = 50
        config.postProcessing.brightnessFilter.maxBrightness = 200
    elif case == "brightness-max-255":
        config.postProcessing.brightnessFilter.minBrightness = -1
        config.postProcessing.brightnessFilter.maxBrightness = 255
    elif case == "hole-filling-off":
        config.postProcessing.holeFilling.enable = False
    elif case == "adaptive-median-off":
        config.postProcessing.adaptiveMedianFilter.enable = False
    else:
        raise ValueError(f"Unknown filter case: {case}")


def get_with_deadline(queue, label, timeout_seconds=30.0):
    """Receive one message without allowing a backend failure to hang the test."""
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        message = queue.tryGet()
        if message is not None:
            return message
        time.sleep(0.005)
    raise TimeoutError(f"Timed out after {timeout_seconds:.0f}s waiting for {label}")


def clone_config(config):
    """Copy every serialized StereoDepthConfig group for runtime replay."""
    clone = dai.StereoDepthConfig()
    clone.algorithmControl = config.algorithmControl
    clone.postProcessing = config.postProcessing
    clone.censusTransform = config.censusTransform
    clone.costMatching = config.costMatching
    clone.costAggregation = config.costAggregation
    clone.confidenceMetrics = config.confidenceMetrics
    clone.setFiltersComputeBackend(config.getFiltersComputeBackend())
    return clone


def timestamp_us(value):
    return None if value is None else round(value.total_seconds() * 1_000_000)


def frame_timing_record(frame):
    return {
        "timestamp_us": timestamp_us(frame.getTimestamp()),
        "timestamp_device_us": timestamp_us(frame.getTimestampDevice()),
        "timestamp_system_us": timestamp_us(frame.getTimestampSystem()),
    }


def normalize_timing(timing, origins):
    return {
        side: {
            name: None if value is None else value - origins[name]
            for name, value in values.items()
        }
        for side, values in timing.items()
    }


def capture_rectified_pairs(device_info, count, fps, alpha):
    pipeline = dai.Pipeline(dai.Device(device_info))
    calibration_handler = pipeline.getCalibrationData()
    translation_cm = np.asarray(
        calibration_handler.getCameraTranslationVector(
            dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, False
        )
    )
    intrinsics = np.asarray(
        calibration_handler.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, *SIZE)
    )
    calibration = {
        "focal_length_px": float(intrinsics[0, 0]),
        "baseline_cm": float(np.linalg.norm(translation_cm)),
    }
    pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setAlphaScaling(alpha)
    left.requestOutput(SIZE, fps=fps).link(stereo.left)
    right.requestOutput(SIZE, fps=fps).link(stereo.right)
    left_queue = stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=True)
    right_queue = stereo.rectifiedRight.createOutputQueue(maxSize=4, blocking=True)

    pairs = []
    timing_origins = None
    with pipeline:
        pipeline.start()
        while len(pairs) < count:
            left_frame = get_with_deadline(left_queue, "RVC2 rectified-left capture")
            right_frame = get_with_deadline(right_queue, "RVC2 rectified-right capture")
            if left_frame.getSequenceNum() != right_frame.getSequenceNum():
                raise RuntimeError("Captured RVC2 rectified streams are out of sync")
            left_transformation = transformation_from_record(
                checked_transformation_record(left_frame.getTransformation())
            )
            right_transformation = transformation_from_record(
                checked_transformation_record(right_frame.getTransformation())
            )
            timing = {
                "left": frame_timing_record(left_frame),
                "right": frame_timing_record(right_frame),
            }
            if timing_origins is None:
                timing_origins = {
                    name: min(
                        value
                        for value in (timing["left"][name], timing["right"][name])
                        if value is not None
                    )
                    for name in timing["left"]
                    if timing["left"][name] is not None or timing["right"][name] is not None
                }
            pairs.append(
                ReplayPair(
                    left_frame.getFrame().copy(),
                    right_frame.getFrame().copy(),
                    left_transformation,
                    right_transformation,
                    normalize_timing(timing, timing_origins),
                )
            )
    return pairs, calibration


def make_frame(image, transformation, timing, sequence, socket):
    frame = dai.ImgFrame()
    frame.setData(image.reshape(-1))
    frame.setTimestamp(datetime.timedelta(microseconds=timing["timestamp_us"]))
    frame.setTimestampDevice(datetime.timedelta(microseconds=timing["timestamp_device_us"]))
    if timing["timestamp_system_us"] is not None:
        frame.setTimestampSystem(datetime.timedelta(microseconds=timing["timestamp_system_us"]))
    frame.setSequenceNum(sequence)
    frame.setInstanceNum(socket)
    frame.setType(dai.ImgFrame.Type.RAW8)
    frame.setWidth(image.shape[1])
    frame.setHeight(image.shape[0])
    frame.setStride(image.shape[1])
    frame.setTransformation(transformation)
    return frame


def create_replay_pipeline(
    device_info, platform, preset, width, filter_case, output_names, runtime_config, in_flight, calibration
):
    pipeline = dai.Pipeline(dai.Device(device_info))
    stereo = pipeline.create(dai.node.StereoDepth)
    if platform == dai.XLinkPlatform.X_LINK_RVC4:
        backends = dai.StereoDepthProperties.StereoBackend
        stereo.setStereoBackend(backends.RVC2)
    stereo.setDefaultProfilePreset(PRESETS[preset])
    stereo.initialConfig.costMatching.disparityWidth = getattr(
        dai.StereoDepthConfig.CostMatching.DisparityWidth, f"DISPARITY_{width}"
    )
    if platform == dai.XLinkPlatform.X_LINK_RVC4 and calibration is not None:
        # The oracle must retain the physical RVC2's native calibration. Give
        # RVC4 those same depth-conversion constants without changing matcher
        # thresholds on the oracle device.
        stereo.setBaseline(calibration["baseline_cm"])
        stereo.setFocalLength(calibration["focal_length_px"])
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)
    runtime_value = None
    config_queue = None
    if runtime_config and filter_case != "native":
        runtime_value = clone_config(stereo.initialConfig)
        configure_case(runtime_value, filter_case)
        config_queue = stereo.inputConfig.createInputQueue(maxSize=1, blocking=True)
    else:
        configure_case(stereo.initialConfig, filter_case)
    return (
        pipeline,
        stereo.left.createInputQueue(maxSize=in_flight, blocking=True),
        stereo.right.createInputQueue(maxSize=in_flight, blocking=True),
        {
            name: getattr(stereo, name).createOutputQueue(maxSize=in_flight, blocking=True)
            for name in output_names
        },
        config_queue,
        runtime_value,
    )


def run_replay(
    device_info, platform, pairs, preset, width, filter_case, warmup, output_names, runtime_config, in_flight, calibration
):
    pipeline, left_queue, right_queue, output_queues, config_queue, runtime_value = create_replay_pipeline(
        device_info, platform, preset, width, filter_case, output_names, runtime_config, in_flight, calibration
    )
    if config_queue is not None and warmup < 2:
        raise ValueError("Runtime StereoDepth configuration requires at least two warmup frames")
    outputs = {name: [] for name in output_names}
    measured_arrivals = []
    with pipeline:
        pipeline.start()
        config_sent = config_queue is None
        next_send = 0
        next_receive = 0
        while next_receive < len(pairs):
            allowed_in_flight = in_flight if config_sent else 1
            while next_send < len(pairs) and next_send - next_receive < allowed_in_flight:
                pair = pairs[next_send]
                left_queue.send(
                    make_frame(
                        pair.left,
                        pair.left_transformation,
                        pair.timing["left"],
                        next_send,
                        dai.CameraBoardSocket.CAM_B,
                    )
                )
                right_queue.send(
                    make_frame(
                        pair.right,
                        pair.right_transformation,
                        pair.timing["right"],
                        next_send,
                        dai.CameraBoardSocket.CAM_C,
                    )
                )
                next_send += 1
            for name, output_queue in output_queues.items():
                output = get_with_deadline(
                    output_queue,
                    f"{platform.name} {preset} D{width} {filter_case} {name} frame {next_receive}",
                )
                if output.getSequenceNum() != next_receive:
                    raise RuntimeError(
                        f"Replay {name} sequence {output.getSequenceNum()} != input {next_receive}"
                    )
                if next_receive >= warmup:
                    outputs[name].append(output.getFrame().copy())
            if not config_sent:
                # Let the node consume its initial properties before sending a
                # runtime update. Sending at pipeline start races the initial
                # config and can produce a transient oracle that later reverts.
                config_queue.send(runtime_value)
                config_sent = True
            if next_receive >= warmup:
                measured_arrivals.append(time.monotonic())
            next_receive += 1
    measured_count = len(measured_arrivals)
    fps = 0.0 if measured_count < 2 else (measured_count - 1) / (measured_arrivals[-1] - measured_arrivals[0])
    return outputs, fps


def config_key(preset, width, filter_case):
    return f"{preset}__d{width}__{filter_case}"


def oracle_manifest_key(output_name):
    # Schema-v2 disparity corpora predate multi-output replay and use "oracles".
    return "oracles" if output_name == "disparity" else f"{output_name}_oracles"


def oracle_filename(output_name, key):
    # Preserve legacy disparity filenames while making every added output explicit.
    return f"oracle__{key}.npy" if output_name == "disparity" else f"oracle__{output_name}__{key}.npy"


def validate_outputs(outputs, expected_count, label, output_name, expected_signature=None):
    if len(outputs) != expected_count:
        raise RuntimeError(f"{label}: received {len(outputs)} outputs, expected {expected_count}")
    signature = {"dtype": str(outputs[0].dtype), "shape": list(outputs[0].shape)}
    if signature["dtype"] not in OUTPUT_DTYPES[output_name] or len(signature["shape"]) != 2:
        raise RuntimeError(f"{label}: invalid {output_name} signature {signature}")
    for index, output in enumerate(outputs):
        if str(output.dtype) != signature["dtype"] or list(output.shape) != signature["shape"]:
            raise RuntimeError(f"{label}: output {index} has inconsistent dtype or shape")
    if expected_signature is not None and signature != expected_signature:
        raise RuntimeError(f"{label}: output signature {signature} != RVC2 oracle {expected_signature}")
    return signature


def load_oracle(corpus, record, expected_count, label, output_name):
    outputs = np.load(corpus / record["file"])
    if array_sha256(outputs) != record["sha256"]:
        raise RuntimeError(f"{label}: {output_name} oracle hash mismatch")
    if "frames" in record and record["frames"] != expected_count:
        raise RuntimeError(
            f"{label}: {output_name} oracle record has {record['frames']} frames, expected {expected_count}"
        )
    signature = validate_outputs(
        outputs,
        expected_count,
        f"{label} RVC2 {output_name} oracle",
        output_name,
        record["signature"],
    )
    return outputs, signature


def save_oracle(corpus, output_name, key, outputs, signature):
    filename = oracle_filename(output_name, key)
    output_array = np.stack(outputs)
    np.save(corpus / filename, output_array)
    return {
        "file": filename,
        "sha256": array_sha256(output_array),
        "signature": signature,
        "frames": len(outputs),
    }


def load_corpus(path):
    with (path / "manifest.json").open(encoding="utf-8") as manifest_file:
        manifest = json.load(manifest_file)
    if manifest.get("schema_version") not in (2, 3, 4):
        raise RuntimeError("Corpus lacks complete per-frame ImgTransformation metadata; recapture it with this script")

    with np.load(path / "inputs.npz") as inputs:
        left_inputs = inputs["left"].copy()
        right_inputs = inputs["right"].copy()
    capture = manifest["capture"]
    transformations = capture["transformations"]
    if len(left_inputs) != len(right_inputs) or len(left_inputs) != len(transformations):
        raise RuntimeError("Corpus input and transformation counts differ")
    timings = capture.get("timings")
    if timings is None:
        # Schema v2 did not retain timing. Keep old corpora readable, using the
        # declared capture cadence rather than the former hard-coded 10 ms.
        interval_us = round(1_000_000 / capture["fps"])
        timings = [
            {
                side: {
                    "timestamp_us": index * interval_us,
                    "timestamp_device_us": index * interval_us,
                    "timestamp_system_us": None,
                }
                for side in ("left", "right")
            }
            for index in range(len(left_inputs))
        ]
    if len(timings) != len(left_inputs):
        raise RuntimeError("Corpus input and timing counts differ")

    pairs = []
    for index, (left, right, records, timing) in enumerate(
        zip(left_inputs, right_inputs, transformations, timings)
    ):
        hashes = capture["input_hashes"][index]
        if array_sha256(left) != hashes["left"] or array_sha256(right) != hashes["right"]:
            raise RuntimeError(f"Corpus input hash mismatch at frame {index}")
        left_transformation = transformation_from_record(records["left"])
        right_transformation = transformation_from_record(records["right"])
        if (
            transformation_record(left_transformation) != records["left"]
            or transformation_record(right_transformation) != records["right"]
        ):
            raise RuntimeError(f"Corpus transformation reconstruction mismatch at frame {index}")
        pairs.append(ReplayPair(left, right, left_transformation, right_transformation, timing))
    return manifest, pairs


def write_manifest(path, manifest):
    with (path / "manifest.json").open("w", encoding="utf-8") as manifest_file:
        json.dump(manifest, manifest_file, indent=2)


def main():
    args = parse_args()
    rvc4 = None if args.oracle_only else find_device(args.rvc4_device, dai.XLinkPlatform.X_LINK_RVC4)
    configurations = [
        (preset, width, filter_case)
        for preset in args.preset
        for width in args.width
        for filter_case in args.filter_case
        if filter_case == "native" or preset in args.filter_preset
    ]
    rvc2_rates = {}
    oracle_records = {name: {} for name in args.output}
    manifest = None

    if args.rvc4_only:
        manifest, pairs = load_corpus(args.corpus)
        calibration = manifest["capture"].get("calibration")
        if calibration is None and "depth" in args.output:
            raise RuntimeError("Corpus lacks the physical RVC2 depth calibration; recapture it with this script")
        args.warmup = manifest["capture"]["warmup"]
        args.in_flight = manifest["capture"].get("replay_in_flight", args.in_flight)
        args.runtime_config = manifest["capture"].get("runtime_filter_configs", args.runtime_config)
        configurations = [(item["preset"], item["width"], item["filter_case"]) for item in manifest["configurations"]]
        rvc2_rates = manifest["rvc2_fps"]
        for output_name in args.output:
            manifest_key = oracle_manifest_key(output_name)
            if manifest_key not in manifest:
                raise RuntimeError(
                    f"Corpus has no {output_name} oracle records; recapture it with --output {output_name}"
                )
            oracle_records[output_name] = manifest[manifest_key]
        print(f"RVC4={rvc4.name} ({rvc4.deviceId}), corpus={args.corpus}")
    else:
        rvc2 = find_device(args.rvc2_device, dai.XLinkPlatform.X_LINK_MYRIAD_X)
        print(f"RVC2={rvc2.name} ({rvc2.deviceId})" + ("" if rvc4 is None else f" RVC4={rvc4.name} ({rvc4.deviceId})"))
        pairs, calibration = capture_rectified_pairs(rvc2, args.frames + args.warmup, args.capture_fps, args.alpha)
        if args.corpus is not None:
            args.corpus.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                args.corpus / "inputs.npz",
                left=np.stack([pair.left for pair in pairs]),
                right=np.stack([pair.right for pair in pairs]),
            )
            manifest = {
                "schema_version": 4,
                "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "runtime": runtime_record(),
                "rvc2_device": device_record(rvc2),
                "capture": {
                    "size": list(SIZE),
                    "fps": args.capture_fps,
                    "alpha": args.alpha,
                    "warmup": args.warmup,
                    "measured_frames": args.frames,
                    "replay_in_flight": args.in_flight,
                    "runtime_filter_configs": args.runtime_config,
                    "calibration": calibration,
                    "input_dtype": str(pairs[0].left.dtype),
                    "input_shape": list(pairs[0].left.shape),
                    "input_hashes": [{"left": array_sha256(pair.left), "right": array_sha256(pair.right)} for pair in pairs],
                    "transformations": [
                        {
                            "left": transformation_record(pair.left_transformation),
                            "right": transformation_record(pair.right_transformation),
                        }
                        for pair in pairs
                    ],
                    "timings": [pair.timing for pair in pairs],
                },
                "configurations": [
                    {"preset": preset, "width": width, "filter_case": filter_case}
                    for preset, width, filter_case in configurations
                ],
                "rvc2_fps": rvc2_rates,
                "oracles": oracle_records.get("disparity", {}),
                "rvc4_runs": [],
            }
            for output_name in args.output:
                if output_name != "disparity":
                    manifest[oracle_manifest_key(output_name)] = oracle_records[output_name]

    failures = []
    run_results = []
    native_rvc2_key = None
    native_rvc2_output = None
    expected_count = len(pairs) - args.warmup
    for preset, width, filter_case in configurations:
        key = config_key(preset, width, filter_case)
        label = f"preset={preset} width=D{width} filters={filter_case}"
        if args.rvc4_only:
            rvc2_outputs = {}
            rvc2_signatures = {}
            for output_name in args.output:
                if key not in oracle_records[output_name]:
                    raise RuntimeError(f"{label}: corpus has no {output_name} oracle")
                rvc2_outputs[output_name], rvc2_signatures[output_name] = load_oracle(
                    args.corpus,
                    oracle_records[output_name][key],
                    expected_count,
                    label,
                    output_name,
                )
            rvc2_fps = rvc2_rates[key]
        else:
            rvc2_outputs, rvc2_fps = run_replay(
                rvc2,
                dai.XLinkPlatform.X_LINK_MYRIAD_X,
                pairs,
                preset,
                width,
                filter_case,
                args.warmup,
                args.output,
                args.runtime_config,
                args.in_flight,
                calibration,
            )
            rvc2_signatures = {
                output_name: validate_outputs(
                    rvc2_outputs[output_name], expected_count, f"{label} RVC2 {output_name}", output_name
                )
                for output_name in args.output
            }
            rvc2_rates[key] = rvc2_fps
            if args.corpus is not None:
                for output_name in args.output:
                    oracle_records[output_name][key] = save_oracle(
                        args.corpus,
                        output_name,
                        key,
                        rvc2_outputs[output_name],
                        rvc2_signatures[output_name],
                    )
        native_key = (preset, width)
        ablation_changed = None
        comparison_output = "disparity" if "disparity" in args.output else args.output[0]
        if filter_case == "native":
            native_rvc2_key = native_key
            native_rvc2_output = rvc2_outputs[comparison_output]
        else:
            if native_key != native_rvc2_key:
                raise RuntimeError(f"{label}: native configuration must run before filter ablations")
            ablated = rvc2_outputs[comparison_output]
            ablation_changed = len(native_rvc2_output) != len(ablated) or any(
                left.shape != right.shape or np.any(left != right)
                for left, right in zip(native_rvc2_output, ablated)
            )
            if args.require_conclusive_filters and filter_case in CONCLUSIVE_FILTER_CASES and not ablation_changed:
                failures.append(
                    f"{label}: actual-RVC2 {comparison_output} is unchanged; filter ablation is inconclusive"
                )
            if args.require_conclusive_filters and filter_case in INERT_FILTER_CASES and ablation_changed:
                failures.append(
                    f"{label}: actual-RVC2 {comparison_output} changed for an RVC2-inert compatibility control"
                )
        if args.oracle_only:
            print(
                f"{label}: saved {expected_count} actual-RVC2 "
                f"{'/'.join(args.output)} oracle frames at {rvc2_fps:.3f} FPS"
            )
            continue

        rvc4_outputs, rvc4_fps = run_replay(
            rvc4,
            dai.XLinkPlatform.X_LINK_RVC4,
            pairs,
            preset,
            width,
            filter_case,
            args.warmup,
            args.output,
            args.runtime_config,
            args.in_flight,
            calibration,
        )
        output_results = {}
        comparison_summaries = []
        for output_name in args.output:
            rvc4_signature = validate_outputs(
                rvc4_outputs[output_name],
                expected_count,
                f"{label} RVC4 {output_name}",
                output_name,
                rvc2_signatures[output_name],
            )
            mismatches = sum(
                np.count_nonzero(left != right)
                for left, right in zip(rvc2_outputs[output_name], rvc4_outputs[output_name])
            )
            pixels = sum(frame.size for frame in rvc2_outputs[output_name])
            comparison_summaries.append(f"{output_name}={mismatches}/{pixels}")
            if mismatches:
                failures.append(f"{label}: {mismatches} {output_name} pixels differ")
            output_results[output_name] = {
                "mismatched_pixels": int(mismatches),
                "compared_pixels": int(pixels),
                "rvc4_sha256": array_sha256(np.stack(rvc4_outputs[output_name])),
                "signature": rvc4_signature,
            }
        ratio = rvc4_fps / rvc2_fps
        print(
            f"{label}: " + ", ".join(comparison_summaries) + ", "
            f"RVC2={rvc2_fps:.3f} FPS, RVC4={rvc4_fps:.3f} FPS, ratio={ratio:.3f}"
        )
        if rvc4_fps < args.min_rvc4_fps:
            failures.append(f"{label}: RVC4 {rvc4_fps:.3f} FPS < {args.min_rvc4_fps:.3f}")
        if ratio < args.min_rvc4_rvc2_ratio:
            failures.append(f"{label}: RVC4/RVC2 ratio {ratio:.3f} < {args.min_rvc4_rvc2_ratio:.3f}")
        run_result = {
            "configuration": key,
            "rvc4_fps": rvc4_fps,
            "rvc2_fps": rvc2_fps,
            "outputs": output_results,
        }
        if ablation_changed is not None:
            run_result["rvc2_ablation_changed_output"] = ablation_changed
        # Retain the original result fields for disparity-only schema-v2 consumers.
        if "disparity" in output_results:
            run_result.update(output_results["disparity"])
        run_results.append(run_result)

    if manifest is not None:
        if rvc4 is not None:
            manifest["rvc4_runs"].append(
                {
                    "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    "device": device_record(rvc4),
                    "runtime": runtime_record(),
                    "results": run_results,
                }
            )
        write_manifest(args.corpus, manifest)

    if failures:
        raise RuntimeError("Parity test failed:\n  " + "\n  ".join(failures))
    print(
        "PASS: actual-RVC2 corpus saved"
        if args.oracle_only
        else f"PASS: all compared {'/'.join(args.output)} buffers are exactly equal"
    )


if __name__ == "__main__":
    main()
