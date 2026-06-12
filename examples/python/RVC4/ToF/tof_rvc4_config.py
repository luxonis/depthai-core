#!/usr/bin/env python3
"""
RVC4 ToF configuration showcase — exposes and prints every RVC4-relevant ToF option.

Uses the unified dai.node.ToF API (no tofBaseNode). Auto-creates Camera unless
--manual-camera is set.

Examples:
    # Print resolved config only (no device stream)
    python3 tof_rvc4_config.py --print-only

    # Default pipeline with live depth preview
    python3 tof_rvc4_config.py

    # Full IPP tuning + extra outputs
    python3 tof_rvc4_config.py \\
        --socket CAM_D --mode F3_FULL --fps 10 --preset MID_RANGE \\
        --bilateral true --tnr true --fpc true --r2p false \\
        --amplitude --confidence --ambient

    # Bypass all IPP filters
    python3 tof_rvc4_config.py --preset OFF --no-ipp-filters
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import depthai as dai
import numpy as np

# Depth/amplitude output size after IPP — use tof.getOutputResolution()
# Raw superframe for manual Camera — use tof.getRawResolution()
SENSOR_MODE_OUTPUT_RESOLUTION = {
    "F1_FULL": (804, 672),
    "F2_FULL": (804, 672),
    "F3_FULL": (804, 672),
    "F2_BINNING_2X2": (402, 336),
    "F3_BINNING_2X2": (402, 336),
}

# Raw superframe size — only needed for manual Camera.build(sensorResolution=...)
SENSOR_MODE_RAW_RESOLUTION = {
    "F1_FULL": (1344, 2420),
    "F2_FULL": (1344, 4832),
    "F3_FULL": (1344, 7244),
    "F2_BINNING_2X2": (672, 2420),
    "F3_BINNING_2X2": (672, 3626),
}

PRESET_PHASE_UNWRAP = {
    "OFF": 10000,
    "LOW_RANGE": 50,
    "MID_RANGE": 75,
    "HIGH_RANGE": 130,
}


def _parse_optional_bool(value: str) -> bool:
    v = value.strip().lower()
    if v in {"1", "true", "yes", "on"}:
        return True
    if v in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"expected true/false, got {value!r}")


@dataclass
class Rvc4ToFConfig:
    board_socket: str
    sensor_mode: str
    sensor_resolution: tuple[int, int]
    fps: Optional[float]
    preset: str
    phase_unwrap_error_threshold: int
    enable_bilateral_filter: Optional[bool]
    bilateral_std_factor: Optional[float]
    bilateral_filter_kernel_size: Optional[int]
    enable_temporal_noise_reduction: Optional[bool]
    tnr_max_gain: Optional[float]
    tnr_std_factor: Optional[float]
    enable_flying_pixel_correction: Optional[bool]
    fp_depth_threshold: Optional[float]
    fp_min_depth_occurrence: Optional[float]
    enable_radial_to_perp: Optional[bool]
    auto_camera: bool
    outputs: list[str]


def _socket(name: str) -> dai.CameraBoardSocket:
    return getattr(dai.CameraBoardSocket, name)


def _sensor_mode(name: str) -> dai.ToFSensorMode:
    return getattr(dai.ToFSensorMode, name)


def _preset(name: str) -> dai.ToFPreset:
    return getattr(dai.ToFPreset, name)


def _fmt(value: Any) -> str:
    if value is None:
        return "unset (IPP default)"
    return str(value)


def _output_list(args: argparse.Namespace) -> list[str]:
    outputs = ["depth"]
    if args.amplitude:
        outputs.append("amplitude")
    if args.confidence:
        outputs.append("confidence")
    if args.ambient:
        outputs.append("intensity")
    return outputs


def _dry_run_config(args: argparse.Namespace) -> Rvc4ToFConfig:
    preset_name = "OFF" if args.no_ipp_filters else args.preset
    return Rvc4ToFConfig(
        board_socket=args.socket,
        sensor_mode=args.mode,
        sensor_resolution=SENSOR_MODE_OUTPUT_RESOLUTION[args.mode],
        fps=args.fps,
        preset=preset_name,
        phase_unwrap_error_threshold=args.phase_unwrap
        if args.phase_unwrap is not None
        else PRESET_PHASE_UNWRAP[preset_name],
        enable_bilateral_filter=False if args.no_ipp_filters else args.bilateral,
        bilateral_std_factor=args.bilateral_std,
        bilateral_filter_kernel_size=args.bilateral_kernel,
        enable_temporal_noise_reduction=False if args.no_ipp_filters else args.tnr,
        tnr_max_gain=args.tnr_max_gain,
        tnr_std_factor=args.tnr_std,
        enable_flying_pixel_correction=False if args.no_ipp_filters else args.fpc,
        fp_depth_threshold=args.fp_depth_th,
        fp_min_depth_occurrence=args.fp_min_occ,
        enable_radial_to_perp=False if args.no_ipp_filters else args.r2p,
        auto_camera=not args.manual_camera,
        outputs=_output_list(args),
    )


def _apply_ipp_args(cfg: dai.ToFConfig, args: argparse.Namespace) -> None:
    if args.phase_unwrap is not None:
        cfg.phaseUnwrapErrorThreshold = int(args.phase_unwrap)

    if args.no_ipp_filters:
        cfg.setToFPreset(dai.ToFPreset.OFF)
        return

    if args.bilateral is not None:
        cfg.enableBilateralFilter = args.bilateral
    if args.bilateral_std is not None:
        cfg.bilateralStdFactor = float(args.bilateral_std)
    if args.bilateral_kernel is not None:
        cfg.bilateralFilterKernelSize = int(args.bilateral_kernel)

    if args.tnr is not None:
        cfg.enableTemporalNoiseReduction = args.tnr
    if args.tnr_max_gain is not None:
        cfg.tnrMaxGain = float(args.tnr_max_gain)
    if args.tnr_std is not None:
        cfg.tnrStdFactor = float(args.tnr_std)

    if args.fpc is not None:
        cfg.enableFlyingPixelCorrection = args.fpc
    if args.fp_depth_th is not None:
        cfg.fpDepthThreshold = float(args.fp_depth_th)
    if args.fp_min_occ is not None:
        cfg.fpMinDepthOccurrence = float(args.fp_min_occ)

    if args.r2p is not None:
        cfg.enableRadialToPerp = args.r2p


def _collect_config(args: argparse.Namespace, cfg: dai.ToFConfig, outputs: list[str], auto_camera: bool) -> Rvc4ToFConfig:
    return Rvc4ToFConfig(
        board_socket=args.socket,
        sensor_mode=args.mode,
        sensor_resolution=SENSOR_MODE_OUTPUT_RESOLUTION[args.mode],
        fps=args.fps,
        preset="OFF" if args.no_ipp_filters else args.preset,
        phase_unwrap_error_threshold=cfg.phaseUnwrapErrorThreshold,
        enable_bilateral_filter=cfg.enableBilateralFilter,
        bilateral_std_factor=cfg.bilateralStdFactor,
        bilateral_filter_kernel_size=cfg.bilateralFilterKernelSize,
        enable_temporal_noise_reduction=cfg.enableTemporalNoiseReduction,
        tnr_max_gain=cfg.tnrMaxGain,
        tnr_std_factor=cfg.tnrStdFactor,
        enable_flying_pixel_correction=cfg.enableFlyingPixelCorrection,
        fp_depth_threshold=cfg.fpDepthThreshold,
        fp_min_depth_occurrence=cfg.fpMinDepthOccurrence,
        enable_radial_to_perp=cfg.enableRadialToPerp,
        auto_camera=auto_camera,
        outputs=outputs,
    )


def print_config_summary(conf: Rvc4ToFConfig) -> None:
    w, h = conf.sensor_resolution
    print("\n=== RVC4 ToF configuration ===")
    print(f"  board socket          : {conf.board_socket}")
    print(f"  sensor mode           : {conf.sensor_mode}  (depth/amplitude output {w} x {h})")
    print(f"  fps                   : {conf.fps if conf.fps is not None else 'auto'}")
    print(f"  IPP preset            : {conf.preset}  (phase unwrap threshold hint: {PRESET_PHASE_UNWRAP.get(conf.preset, '?')})")
    print(f"  auto camera           : {conf.auto_camera}")
    print(f"  outputs               : {', '.join(conf.outputs)}")
    print("\n--- IPP initialConfig (device-side) ---")
    print(f"  phaseUnwrapErrorThreshold     : {conf.phase_unwrap_error_threshold}")
    print(f"  enableBilateralFilter         : {_fmt(conf.enable_bilateral_filter)}")
    print(f"  bilateralStdFactor            : {_fmt(conf.bilateral_std_factor)}")
    print(f"  bilateralFilterKernelSize     : {_fmt(conf.bilateral_filter_kernel_size)}")
    print(f"  enableTemporalNoiseReduction  : {_fmt(conf.enable_temporal_noise_reduction)}")
    print(f"  tnrMaxGain                    : {_fmt(conf.tnr_max_gain)}")
    print(f"  tnrStdFactor                  : {_fmt(conf.tnr_std_factor)}")
    print(f"  enableFlyingPixelCorrection   : {_fmt(conf.enable_flying_pixel_correction)}")
    print(f"  fpDepthThreshold              : {_fmt(conf.fp_depth_threshold)}")
    print(f"  fpMinDepthOccurrence          : {_fmt(conf.fp_min_depth_occurrence)}")
    print(f"  enableRadialToPerp            : {_fmt(conf.enable_radial_to_perp)}")
    print("\n--- RVC2-only fields (ignored on RVC4) ---")
    print("  median, enableBurstMode, enableDistortionCorrection, enablePhaseShuffleTemporalFilter,")
    print("  enableFPPNCorrection, enableOpticalCorrection, enableTemperatureCorrection,")
    print("  enableWiggleCorrection, enablePhaseUnwrapping, phaseUnwrappingLevel")
    print("===\n")


def colorize_depth(frame: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
    invalid = frame == 0
    out = np.zeros((*frame.shape, 3), dtype=np.uint8)
    valid = frame[frame > 0]
    if valid.size == 0:
        return out
    lo = float(np.percentile(valid, 3)) if depth_min <= 0 else depth_min
    hi = float(np.percentile(valid, 95)) if depth_max <= 0 else depth_max
    if hi <= lo:
        hi = lo + 1.0
    norm = np.clip((frame.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
    colored = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    colored[invalid] = 0
    return colored


def normalize_float(frame: np.ndarray) -> np.ndarray:
    f = frame.astype(np.float32)
    lo, hi = np.min(f), np.max(f)
    if hi <= lo:
        return np.zeros(f.shape, dtype=np.uint8)
    return ((f - lo) / (hi - lo) * 255).astype(np.uint8)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RVC4 ToF config showcase")

    p.add_argument("--device", "-d", default=None, help="Device IP or MxId (DEPTHAI_DEVICE_NAME_LIST)")
    p.add_argument("--print-only", action="store_true", help="Print config and exit (no streaming)")
    p.add_argument("--manual-camera", action="store_true", help="Create Camera manually and link rawInput (skip auto-camera)")

    build = p.add_argument_group("build")
    build.add_argument("--socket", "-s", default="AUTO", choices=["AUTO", "CAM_A", "CAM_B", "CAM_C", "CAM_D"])
    build.add_argument(
        "--mode",
        default="F3_FULL",
        choices=list(SENSOR_MODE_OUTPUT_RESOLUTION.keys()),
        help="ToFSensorMode (depth/amplitude output size)",
    )
    build.add_argument("--fps", type=float, default=10.0, help="Sensor FPS")
    build.add_argument(
        "--preset",
        default="MID_RANGE",
        choices=list(PRESET_PHASE_UNWRAP.keys()),
        help="ToFPreset — IPP phase-unwrap / filter profile",
    )

    ipp = p.add_argument_group("IPP initialConfig overrides (optional)")
    ipp.add_argument("--phase-unwrap", type=int, default=None, dest="phase_unwrap", help="phaseUnwrapErrorThreshold")
    ipp.add_argument("--no-ipp-filters", action="store_true", help="Set ToFPreset.OFF (bypass bilateral/TNR/FPC/R2P)")
    ipp.add_argument("--bilateral", type=_parse_optional_bool, default=None, help="enableBilateralFilter true/false")
    ipp.add_argument("--bilateral-std", type=float, default=None, dest="bilateral_std")
    ipp.add_argument("--bilateral-kernel", type=int, default=None, dest="bilateral_kernel")
    ipp.add_argument("--tnr", type=_parse_optional_bool, default=None, help="enableTemporalNoiseReduction")
    ipp.add_argument("--tnr-max-gain", type=float, default=None, dest="tnr_max_gain")
    ipp.add_argument("--tnr-std", type=float, default=None, dest="tnr_std")
    ipp.add_argument("--fpc", type=_parse_optional_bool, default=None, help="enableFlyingPixelCorrection")
    ipp.add_argument("--fp-depth-th", type=float, default=None, dest="fp_depth_th")
    ipp.add_argument("--fp-min-occ", type=float, default=None, dest="fp_min_occ")
    ipp.add_argument("--r2p", type=_parse_optional_bool, default=None, help="enableRadialToPerp")

    view = p.add_argument_group("live preview")
    view.add_argument("--amplitude", action="store_true")
    view.add_argument("--confidence", action="store_true")
    view.add_argument("--ambient", action="store_true", help="intensity / ambient")
    view.add_argument("--depth-min", type=float, default=0.0)
    view.add_argument("--depth-max", type=float, default=6000.0)

    return p.parse_args()


def setup_tof(pipeline: dai.Pipeline, args: argparse.Namespace) -> dai.node.ToF:
    device = pipeline.getDefaultDevice()
    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError(f"This script is RVC4-only (connected device: {device.getPlatform()})")

    tof = pipeline.create(dai.node.ToF)

    preset = _preset("OFF" if args.no_ipp_filters else args.preset)
    tof.build(
        _socket(args.socket),
        sensorMode=_sensor_mode(args.mode),
        fps=args.fps,
        preset=preset,
    )
    _apply_ipp_args(tof.initialConfig, args)

    if args.manual_camera:
        cam = pipeline.create(dai.node.Camera)
        cam.setSensorType(dai.CameraSensorType.TOF)
        cam.build(tof.getBoardSocket(), sensorResolution=tof.getRawResolution(), sensorFps=args.fps)
        cam.raw.link(tof.rawInput)

    return tof


def main() -> int:
    args = parse_args()
    if args.device:
        os.environ["DEPTHAI_DEVICE_NAME_LIST"] = args.device

    if args.print_only:
        print_config_summary(_dry_run_config(args))
        print("Available ToFSensorMode values:", ", ".join(SENSOR_MODE_OUTPUT_RESOLUTION))
        print("Available ToFPreset values:", ", ".join(PRESET_PHASE_UNWRAP))
        return 0

    pipeline = dai.Pipeline()
    tof = setup_tof(pipeline, args)
    conf = _collect_config(args, tof.initialConfig, _output_list(args), auto_camera=not args.manual_camera)
    print_config_summary(conf)

    depth_q = tof.depth.createOutputQueue(maxSize=4, blocking=False)
    amp_q = tof.amplitude.createOutputQueue(maxSize=4, blocking=False) if args.amplitude else None
    conf_q = tof.confidence.createOutputQueue(maxSize=4, blocking=False) if args.confidence else None
    amb_q = tof.intensity.createOutputQueue(maxSize=4, blocking=False) if args.ambient else None

    with pipeline as p:
        p.start()
        auto_cam = tof.getCamera()
        if auto_cam is not None:
            print(f"Auto-created Camera on {tof.getBoardSocket()}")
        elif args.manual_camera:
            print("Using external Camera linked to rawInput")
        print("Streaming. Press 'q' to quit.")
        while p.isRunning():
            depth = depth_q.tryGet()
            if depth is not None:
                img = colorize_depth(depth.getFrame(), args.depth_min, args.depth_max)
                cv2.imshow("depth (IPP)", img)

            if amp_q:
                amp = amp_q.tryGet()
                if amp is not None:
                    cv2.imshow("amplitude", normalize_float(amp.getFrame()))

            if conf_q:
                c = conf_q.tryGet()
                if c is not None:
                    cv2.imshow("confidence", normalize_float(c.getFrame()))

            if amb_q:
                a = amb_q.tryGet()
                if a is not None:
                    cv2.imshow("ambient", normalize_float(a.getFrame()))

            if cv2.waitKey(1) == ord("q"):
                break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(0)
