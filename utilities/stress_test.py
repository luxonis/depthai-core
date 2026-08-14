#!/usr/bin/env python3
"""Exercise the current DepthAI pipeline API with all cameras on a device.

The script intentionally creates several concurrent workloads: camera output,
video encoding, edge detection, stereo depth, optional object detection, and
system logging.  It uses the post-XLink pipeline API, where node outputs and
inputs create their host queues directly.

Controls while running:
    q       quit
    a / d   decrease / increase IR dot-projector intensity
    s / w   decrease / increase IR flood-light intensity
    k / l   decrease / increase manual exposure ISO
    i / o   decrease / increase manual exposure time
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import signal
import time
from typing import Any, Optional

import cv2
import depthai as dai
import numpy as np


CAMERA_FPS = 20.0
ENCODER_FPS = 10.0
NN_FPS = 15.0
TOF_FPS = 30.0
CAMERA_SIZE = (1280, 800)

DOT_STEP = 0.05
FLOOD_STEP = 0.05


@dataclass
class Stream:
    name: str
    queue: Any
    kind: str


@dataclass
class PipelineContext:
    detection_frame_name: Optional[str] = None
    labels: list[str] = field(default_factory=list)


def on_exit(_sig: int, _frame: Any) -> None:
    cv2.destroyAllWindows()
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, on_exit)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def print_system_information(info: dai.SystemInformation) -> None:
    mib = 1024.0 * 1024.0
    print(f"DDR used / total - {info.ddrMemoryUsage.used / mib:.2f} / {info.ddrMemoryUsage.total / mib:.2f} MiB")
    print(f"CMX used / total - {info.cmxMemoryUsage.used / mib:.2f} / {info.cmxMemoryUsage.total / mib:.2f} MiB")
    print(f"Leon CSS heap - {info.leonCssMemoryUsage.used / mib:.2f} / {info.leonCssMemoryUsage.total / mib:.2f} MiB")
    print(f"Leon MSS heap - {info.leonMssMemoryUsage.used / mib:.2f} / {info.leonMssMemoryUsage.total / mib:.2f} MiB")
    temperature = info.chipTemperature
    print(
        "Temperature - average: "
        f"{temperature.average:.2f}, css: {temperature.css:.2f}, "
        f"mss: {temperature.mss:.2f}, upa: {temperature.upa:.2f}, "
        f"dss: {temperature.dss:.2f}"
    )
    print(
        "CPU - Leon CSS: "
        f"{info.leonCssCpuUsage.average * 100:.2f}%, "
        f"Leon MSS: {info.leonMssCpuUsage.average * 100:.2f}%"
    )


def print_system_information_rvc4(info: dai.SystemInformationRVC4) -> None:
    mib = 1024.0 * 1024.0
    temperature = info.chipTemperature
    print(f"DDR used / total - {info.ddrMemoryUsage.used / mib:.2f} / {info.ddrMemoryUsage.total / mib:.2f} MiB")
    print(f"CPU average - {info.cpuAvgUsage.average * 100:.2f}%")
    print(f"Process memory - {info.processMemoryUsage / mib:.2f} MiB")
    print(f"Process CPU - {info.processCpuAvgUsage.average * 100:.2f}%")
    print(
        "Temperature - average: "
        f"{temperature.average:.2f}, cpuss: {temperature.cpuss:.2f}, "
        f"gpuss: {temperature.gpuss:.2f}, mdmss: {temperature.mdmss:.2f}, "
        f"video: {temperature.video:.2f}, ddr: {temperature.ddr:.2f}, "
        f"camera: {temperature.camera:.2f}"
    )


def colorize_depth(frame: np.ndarray) -> np.ndarray:
    """Convert a depth image to a robust, displayable color image."""
    valid = frame[frame > 0]
    if valid.size == 0:
        return np.zeros((*frame.shape[:2], 3), dtype=np.uint8)

    minimum = float(np.percentile(valid, 1))
    maximum = float(np.percentile(valid, 99))
    if maximum <= minimum:
        maximum = minimum + 1.0

    normalized = np.clip((frame.astype(np.float32) - minimum) * 255.0 / (maximum - minimum), 0, 255)
    colored = cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_JET)
    colored[frame == 0] = 0
    return colored


def add_detection_overlay(frame: np.ndarray, packet: dai.ImgDetections, labels: list[str]) -> None:
    height, width = frame.shape[:2]
    for detection in packet.detections:
        x1 = max(0, min(width - 1, int(detection.xmin * width)))
        y1 = max(0, min(height - 1, int(detection.ymin * height)))
        x2 = max(0, min(width - 1, int(detection.xmax * width)))
        y2 = max(0, min(height - 1, int(detection.ymax * height)))
        label = labels[detection.label] if 0 <= detection.label < len(labels) else str(detection.label)
        cv2.putText(
            frame,
            f"{label} {detection.confidence:.0%}",
            (x1 + 8, max(20, y1 + 20)),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            (255, 255, 255),
        )
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)


def build_pipeline(
    device: dai.Device, args: argparse.Namespace
) -> tuple[dai.Pipeline, list[Stream], list[Any], Any, PipelineContext]:
    """Build a pipeline using current node build methods and direct queues."""
    pipeline = dai.Pipeline(device)
    streams: list[Stream] = []
    control_queues: list[Any] = []
    context = PipelineContext(labels=[])

    system_logger = pipeline.create(dai.node.SystemLogger)
    system_logger.setRate(1.0)
    system_queue = system_logger.out.createOutputQueue(maxSize=1, blocking=False)

    features = device.getConnectedCameraFeatures()
    cameras: dict[Any, tuple[Any, dai.Node.Output, dai.CameraSensorType]] = {}
    color_camera: Optional[Any] = None
    edge_count = 0

    for feature in features:
        if not feature.supportedTypes:
            print(f"Skipping {feature.socket}: no sensor type reported")
            continue

        sensor_type = feature.supportedTypes[0]
        configs = [(config.width, config.height, config.type) for config in feature.configs]
        print(
            f"{feature.socket}: {feature.sensorName}, max {feature.width}x{feature.height}, "
            f"types {feature.supportedTypes}, configs {configs}"
        )

        if sensor_type == dai.CameraSensorType.TOF:
            tof = pipeline.create(dai.node.ToF).build(
                boardSocket=feature.socket,
                profile=dai.ToFConfig.Profile.MID_RANGE,
                fps=TOF_FPS,
            )
            streams.append(Stream(f"tof_{feature.socket.name}", tof.depth.createOutputQueue(maxSize=1, blocking=False), "tof"))
            continue

        if sensor_type not in (dai.CameraSensorType.COLOR, dai.CameraSensorType.MONO):
            print(f"Skipping {feature.socket}: unsupported sensor type {sensor_type}")
            continue

        camera = pipeline.create(dai.node.Camera).setSensorType(sensor_type).build(
            feature.socket,
            sensorFps=CAMERA_FPS,
        )
        camera_output = camera.requestOutput(CAMERA_SIZE, fps=CAMERA_FPS)
        # print(f"RES: ${feature.configs} ${feature.socket}")
        if sensor_type not in (dai.CameraSensorType.COLOR, dai.CameraSensorType.MONO):
            print(f"Skipping {feature.socket}: {CAMERA_SIZE} output size likely unsupported.")
            continue
        socket_name = feature.socket.name
        streams.append(Stream(f"preview_{socket_name}", camera_output.createOutputQueue(maxSize=2, blocking=False), "image"))
        control_queues.append(camera.inputControl.createInputQueue(maxSize=4, blocking=False))
        cameras[feature.socket] = (camera, camera_output, sensor_type)

        if sensor_type == dai.CameraSensorType.COLOR and color_camera is None:
            color_camera = camera

        encoder = pipeline.create(dai.node.VideoEncoder).build(
            camera_output,
            frameRate=ENCODER_FPS,
            profile=dai.VideoEncoderProperties.Profile.H264_MAIN,
        )
        streams.append(
            Stream(
                f"{socket_name}.encoded",
                encoder.bitstream.createOutputQueue(maxSize=5, blocking=False),
                "encoded",
            )
        )

        if edge_count < args.n_edge_detectors:
            edge_count += 1
            edge_detector = pipeline.create(dai.node.EdgeDetector)
            edge_detector.setMaxOutputFrameSize(CAMERA_SIZE[0] * CAMERA_SIZE[1] * 3)
            camera_output.link(edge_detector.inputImage)
            streams.append(
                Stream(
                    f"{socket_name}.edges",
                    edge_detector.outputImage.createOutputQueue(maxSize=2, blocking=False),
                    "image",
                )
            )

    stereo: Optional[Any] = None
    stereo_left_output: Optional[Any] = None
    stereo_right_output: Optional[Any] = None
    stereo_pairs = device.getStereoPairs()
    if stereo_pairs and not args.no_stereo:
        pair = stereo_pairs[0]
        left_entry = cameras.get(pair.left)
        right_entry = cameras.get(pair.right)
        if left_entry is not None and right_entry is not None:
            stereo_left_output = left_entry[1]
            stereo_right_output = right_entry[1]
            stereo = pipeline.create(dai.node.StereoDepth).build(
                stereo_left_output,
                stereo_right_output,
                dai.node.StereoDepth.PresetMode.HIGH_DETAIL,
            )
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)

            color_sockets = [socket for socket, entry in cameras.items() if entry[2] == dai.CameraSensorType.COLOR]
            align_socket = color_sockets[0] if color_sockets else pair.left
            stereo.setDepthAlign(align_socket)
            streams.append(Stream("stereo depth", stereo.depth.createOutputQueue(maxSize=2, blocking=False), "depth"))
        else:
            print(f"Stereo pair {pair.left}/{pair.right} is not available as a camera output; skipping depth")
    elif args.no_stereo:
        print("--no-stereo set, skipping stereo depth")
    else:
        print("Device has no stereo pair, skipping stereo depth")

    if color_camera is not None and not args.no_nnet:
        model = dai.NNModelDescription("yolov6-nano")
        if stereo is not None:
            network = pipeline.create(dai.node.SpatialDetectionNetwork).build(color_camera, stereo, model, fps=NN_FPS)
            network.setDepthLowerThreshold(100)
            network.setDepthUpperThreshold(5000)
            network.setBoundingBoxScaleFactor(0.5)
        else:
            network = pipeline.create(dai.node.DetectionNetwork).build(color_camera, model, fps=NN_FPS)

        network.setConfidenceThreshold(0.5)
        network.input.setBlocking(False)
        labels = network.getClasses()
        context.labels = list(labels) if labels else []
        streams.append(Stream("detections", network.out.createOutputQueue(maxSize=4, blocking=False), "detections"))
        passthrough_name = f"preview_{color_camera.getBoardSocket().name}"
        # streams.append(Stream(passthrough_name+"_passthrough", network.passthrough.createOutputQueue(maxSize=2, blocking=False), "image"))
        context.detection_frame_name = passthrough_name
    elif color_camera is None:
        print("No color camera found, skipping neural network")
    else:
        print("--no-nnet set, skipping neural network")

    return pipeline, streams, control_queues, system_queue, context


def send_manual_exposure(control_queues: list[Any], exposure: int, iso: int) -> None:
    control = dai.CameraControl()
    control.setManualExposure(exposure, iso)
    for queue in control_queues:
        queue.send(control)


def stress_test(mxid: str = "") -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mxid", default=mxid, help="Device ID, device name, or IP address")
    parser.add_argument("-ne", "--n-edge-detectors", default=0, type=int)
    parser.add_argument("--no-nnet", action="store_true", help="Do not create a detection network")
    parser.add_argument("--no-stereo", action="store_true", help="Do not create stereo depth")
    parser.add_argument("--slow-rampup", action="store_true", help="Ramp IR intensity after the pipeline starts")
    parser.add_argument("--rampup-seconds", type=float, default=5.0)
    args = parser.parse_args()

    device = dai.Device(args.mxid) if args.mxid else dai.Device()
    pipeline, streams, control_queues, system_queue, context = build_pipeline(device, args)

    dot_intensity = 0.5
    flood_intensity = 0.5
    iso = 800
    exposure = 20000

    pipeline.start()

    if args.slow_rampup:
        device.setIrLaserDotProjectorIntensity(0.0)
        device.setIrFloodLightIntensity(0.0)
    else:
        device.setIrLaserDotProjectorIntensity(dot_intensity)
        device.setIrFloodLightIntensity(flood_intensity)

    start_time = time.monotonic()
    ramp_start = start_time if args.slow_rampup else None
    last_ramp_update = 0.0
    last_dot = None
    last_flood = None
    last_frames: dict[str, np.ndarray] = {}

    try:
        print(f"Started on {device.getPlatformAsString()} ({device.getUsbSpeed()})")
        while pipeline.isRunning():
            if ramp_start is not None:
                elapsed = time.monotonic() - ramp_start
                fraction = 1.0 if args.rampup_seconds <= 0 else clamp(elapsed / args.rampup_seconds, 0.0, 1.0)
                now = time.monotonic()
                if now - last_ramp_update >= 0.05 or fraction >= 1.0:
                    dot = dot_intensity * fraction
                    flood = flood_intensity * fraction
                    if last_dot is None or abs(dot - last_dot) >= 1e-3:
                        device.setIrLaserDotProjectorIntensity(dot)
                        last_dot = dot
                    if last_flood is None or abs(flood - last_flood) >= 1e-3:
                        device.setIrFloodLightIntensity(flood)
                        last_flood = flood
                    last_ramp_update = now
                if fraction >= 1.0:
                    ramp_start = None

            for stream in streams:
                packet = stream.queue.tryGet()
                if packet is None:
                    continue
                if stream.kind == "tof" or stream.kind == "depth":
                    last_frames[stream.name] = colorize_depth(packet.getCvFrame())
                elif stream.kind == "image":
                    if isinstance(packet, dai.ImgFrame) and packet.getType() != dai.ImgFrame.Type.BITSTREAM:
                        last_frames[stream.name] = packet.getCvFrame()
                elif stream.kind == "detections":
                    frame_name = context.detection_frame_name
                    frame = last_frames.get(frame_name) if frame_name else None
                    if frame is not None:
                        add_detection_overlay(frame, packet, context.labels or [])

            system_info = system_queue.tryGet()
            if system_info is not None:
                print(f"[{int(time.monotonic() - start_time)}s] USB speed {device.getUsbSpeed()}")
                if isinstance(system_info, dai.SystemInformationRVC4):
                    print_system_information_rvc4(system_info)
                else:
                    print_system_information(system_info)

            for name, frame in last_frames.items():
                cv2.imshow(name, frame)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("a"):
                dot_intensity = clamp(dot_intensity - DOT_STEP, 0.0, 1.0)
                device.setIrLaserDotProjectorIntensity(dot_intensity)
            elif key == ord("d"):
                dot_intensity = clamp(dot_intensity + DOT_STEP, 0.0, 1.0)
                device.setIrLaserDotProjectorIntensity(dot_intensity)
            elif key == ord("s"):
                flood_intensity = clamp(flood_intensity - FLOOD_STEP, 0.0, 1.0)
                device.setIrFloodLightIntensity(flood_intensity)
            elif key == ord("w"):
                flood_intensity = clamp(flood_intensity + FLOOD_STEP, 0.0, 1.0)
                device.setIrFloodLightIntensity(flood_intensity)
            elif key == ord("k"):
                iso = int(clamp(iso - 50, 0, 1600))
                send_manual_exposure(control_queues, exposure, iso)
            elif key == ord("l"):
                iso = int(clamp(iso + 50, 0, 1600))
                send_manual_exposure(control_queues, exposure, iso)
            elif key == ord("i"):
                exposure = int(clamp(exposure - 500, 0, 33000))
                send_manual_exposure(control_queues, exposure, iso)
            elif key == ord("o"):
                exposure = int(clamp(exposure + 500, 0, 33000))
                send_manual_exposure(control_queues, exposure, iso)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        pipeline.wait()


if __name__ == "__main__":
    stress_test()
