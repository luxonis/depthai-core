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


TOF_FPS = 30.0
CAMERA_SIZE = (1280, 800)
MANIP_SIZE = (1280, 720)
MAX_VIDEO_WIDTH = 1920
NN_MODEL = "yolov6-nano"
NN_SHAVES_RVC2 = 6
NEURAL_DEPTH_MODEL = dai.DeviceModelZoo.NEURAL_DEPTH_EXTRA_LARGE
ENCODER_PROFILES = [
    dai.VideoEncoderProperties.Profile.H264_MAIN,
    dai.VideoEncoderProperties.Profile.MJPEG,
    dai.VideoEncoderProperties.Profile.H265_MAIN,
]

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


def load_nn_archive(device: dai.Device, model_name: str) -> dai.NNArchive:
    """Get the model for the platform of the device from the model zoo."""
    description = dai.NNModelDescription(model_name)
    description.platform = device.getPlatformAsString()
    return dai.NNArchive(dai.getModelFromZoo(description))


def limit_nn_shaves(network: Any, archive: dai.NNArchive, num_shaves: int) -> None:
    """Compile the RVC2 superblob for a smaller number of shaves.

    Other nodes in this pipeline use most of the CMX slices. If the blob asks
    for more shaves than the device has free, the pipeline does not start.
    """
    if archive.getModelType() != dai.ModelType.SUPERBLOB:
        return
    network.setNNArchive(archive, numShaves=num_shaves)
    print(f"Neural network blob compiled for {num_shaves} shaves")


def pick_video_size(feature: Any) -> tuple[int, int]:
    """Get the largest sensor configuration that an encoder can still take."""
    sizes = {(config.width, config.height) for config in feature.configs}
    usable = [size for size in sizes if size[0] <= MAX_VIDEO_WIDTH]
    if not usable:
        return min(sizes, key=lambda size: size[0] * size[1])
    return max(usable, key=lambda size: size[0] * size[1])


def add_manip_chain(pipeline: dai.Pipeline, source: dai.Node.Output, count: int) -> None:
    """Chain image manipulation nodes to give the device more work.

    The result stays on the device. Nothing reads the last output, so this
    load does not use the link to the host.
    """
    for index in range(count):
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.addRotateDeg(90 if index % 2 == 0 else -90)
        manip.initialConfig.setOutputSize(*MANIP_SIZE)
        manip.setMaxOutputFrameSize(MANIP_SIZE[0] * MANIP_SIZE[1] * 3)
        source.link(manip.inputImage)
        source = manip.out


def nn_input_type(device: dai.Device, archive: dai.NNArchive) -> dai.ImgFrame.Type:
    """Get the frame type that the model wants at its input."""
    inputs = archive.getConfigV1().model.inputs
    dai_type = inputs[0].preprocessing.daiType if inputs else None
    if dai_type:
        return getattr(dai.ImgFrame.Type, dai_type)
    if device.getPlatform() == dai.Platform.RVC4:
        return dai.ImgFrame.Type.BGR888i
    return dai.ImgFrame.Type.BGR888p


def add_downscale(
    pipeline: dai.Pipeline,
    source: dai.Node.Output,
    size: tuple[int, int],
    frame_type: Optional[dai.ImgFrame.Type] = None,
    resize_mode: Any = dai.ImageManipConfig.ResizeMode.LETTERBOX,
) -> dai.Node.Output:
    """Resize frames on the device with an ImageManip node."""
    manip = pipeline.create(dai.node.ImageManip)
    manip.initialConfig.setOutputSize(size[0], size[1], resize_mode)
    if frame_type is not None:
        manip.initialConfig.setFrameType(frame_type)
    manip.setMaxOutputFrameSize(size[0] * size[1] * 3)
    source.link(manip.inputImage)
    return manip.out


def add_network(pipeline: dai.Pipeline, source: dai.Node.Output, archive: dai.NNArchive, frame_type: dai.ImgFrame.Type) -> Any:
    """Add a detection network that gets its frames through an ImageManip node.

    A camera gives a limited number of outputs, so these networks resize an
    output that already exists instead of asking the camera for a new one.
    Only the first network gets depth, and it takes its frames straight from
    the camera, because the build method also aligns the depth to them.
    """
    size = (archive.getInputWidth(), archive.getInputHeight())
    resized = add_downscale(pipeline, source, size, frame_type, dai.ImageManipConfig.ResizeMode.STRETCH)
    return pipeline.create(dai.node.DetectionNetwork).build(resized, archive)


def build_pipeline(
    device: dai.Device, args: argparse.Namespace
) -> tuple[dai.Pipeline, list[Stream], list[Any], Any, PipelineContext]:
    """Build a pipeline using current node build methods and direct queues."""
    pipeline = dai.Pipeline(device)
    streams: list[Stream] = []
    control_queues: list[Any] = []
    context = PipelineContext()
    is_rvc4 = device.getPlatform() == dai.Platform.RVC4
    if is_rvc4:
        camera_fps = 30.0
        encoder_fps = 30.0
        n_encoders = 2
        n_manips = 2
        n_networks = 3
        full_resolution_color = True
        # An encoder takes the largest configuration that the sensor gives.
        forced_video_size = None
    else:
        # RVC2 has much less compute, so it keeps a small number of nodes.
        camera_fps = 20.0
        encoder_fps = 10.0
        n_encoders = 1
        n_manips = 0
        n_networks = 1
        full_resolution_color = False
        # A larger encoder input starves the other nodes of ISP bandwidth here.
        forced_video_size = CAMERA_SIZE

    # The command line replaces the values for the platform.
    if args.camera_fps is not None:
        camera_fps = args.camera_fps
    if args.encoder_fps is not None:
        encoder_fps = args.encoder_fps
    if args.n_encoders is not None:
        n_encoders = args.n_encoders
    if args.n_manips is not None:
        n_manips = args.n_manips
    if args.n_nnets is not None:
        n_networks = args.n_nnets
    if args.full_res_color is not None:
        full_resolution_color = args.full_res_color

    print(
        f"Load on {device.getPlatformAsString()}: camera {camera_fps} fps, "
        f"{n_encoders} encoder(s) per camera at {encoder_fps} fps, "
        f"{n_manips} manip(s) per camera, {n_networks} network(s), "
        f"full resolution color {full_resolution_color}"
    )

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

        full_color = sensor_type == dai.CameraSensorType.COLOR and full_resolution_color
        camera = pipeline.create(dai.node.Camera).setSensorType(sensor_type).build(
            feature.socket,
            sensorFps=camera_fps,
        )

        # Every camera runs at its full resolution. The color sensor gives
        # frames that are too large for the host, so an ImageManip node on the
        # device makes them smaller. requestFullResolutionOutput keeps away
        # from configurations over 5000x4000, which other nodes cannot share.
        full_output: Optional[dai.Node.Output] = None
        if sensor_type == dai.CameraSensorType.MONO:
            camera_output = camera.requestFullResolutionOutput(fps=camera_fps)
        elif full_color:
            full_output = camera.requestFullResolutionOutput(fps=camera_fps)
            camera_output = add_downscale(pipeline, full_output, CAMERA_SIZE)
            print(f"{feature.socket}: full resolution output, downscaled to {CAMERA_SIZE} for the host")
        else:
            camera_output = camera.requestOutput(CAMERA_SIZE, fps=camera_fps)
        socket_name = feature.socket.name
        streams.append(Stream(f"preview_{socket_name}", camera_output.createOutputQueue(maxSize=2, blocking=False), "image"))
        control_queues.append(camera.inputControl.createInputQueue(maxSize=4, blocking=False))
        cameras[feature.socket] = (camera, camera_output, sensor_type)

        if sensor_type == dai.CameraSensorType.COLOR and color_camera is None:
            color_camera = camera

        # The VideoEncoder accepts only NV12 and GRAY8 frames. A mono camera
        # gives GRAY8, but a color camera gives YUV420p, so ask for a second
        # output in NV12 format. It is also the largest output that an encoder
        # can take, which puts more load on the ISP and on the encoder.
        # An output with no consumer stops the pipeline from starting, so ask
        # for the encoder input only when there is an encoder to take it.
        # Every output of a camera is requested at the rate of the sensor. Two
        # outputs of one sensor at different rates make RVC2 deliver about one
        # frame every five seconds on all of them.
        encoder_frame_rate = min(encoder_fps, camera_fps)
        video_size = forced_video_size or pick_video_size(feature)
        if n_encoders <= 0:
            encoder_input = None
        elif full_output is not None:
            encoder_input = add_downscale(pipeline, full_output, video_size, dai.ImgFrame.Type.NV12)
        elif sensor_type == dai.CameraSensorType.COLOR:
            encoder_input = camera.requestOutput(video_size, dai.ImgFrame.Type.NV12, fps=camera_fps)
        else:
            encoder_input = camera_output

        for index in range(n_encoders):
            encoder_profile = ENCODER_PROFILES[index % len(ENCODER_PROFILES)]
            encoder = pipeline.create(dai.node.VideoEncoder).build(
                encoder_input,
                frameRate=encoder_frame_rate,
                profile=encoder_profile,
            )
            streams.append(
                Stream(
                    f"{socket_name}.{encoder_profile.name.lower()}",
                    encoder.bitstream.createOutputQueue(maxSize=5, blocking=False),
                    "encoded",
                )
            )

        add_manip_chain(pipeline, camera_output, n_manips)

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
    stereo_pairs = device.getStereoPairs()
    if stereo_pairs and not args.no_stereo:
        pair = stereo_pairs[0]
        left_entry = cameras.get(pair.left)
        right_entry = cameras.get(pair.right)
        if left_entry is not None and right_entry is not None:
            if is_rvc4:
                # NeuralDepth rectifies the pair itself and has no depth
                # alignment, so it needs no more configuration.
                stereo = pipeline.create(dai.node.NeuralDepth).build(
                    left_entry[1],
                    right_entry[1],
                    NEURAL_DEPTH_MODEL,
                )
                print(f"Using NeuralDepth with model {NEURAL_DEPTH_MODEL.name}")
            else:
                stereo = pipeline.create(dai.node.StereoDepth).build(
                    left_entry[1],
                    right_entry[1],
                    dai.node.StereoDepth.PresetMode.DEFAULT,
                )
                stereo.setLeftRightCheck(True)
                stereo.setSubpixel(True)

                color_sockets = [socket for socket, entry in cameras.items() if entry[2] == dai.CameraSensorType.COLOR]
                align_socket = color_sockets[0] if color_sockets else pair.left
                stereo.setDepthAlign(align_socket)
                # Aligned depth takes the size of the color frame, and its
                # width must be a multiple of 16. Some sensors, for example the
                # IMX214 with its smallest configuration of 2104x1560, do not
                # give such a width, so the size is set here.
                stereo.setOutputSize(*CAMERA_SIZE)
            streams.append(Stream("stereo depth", stereo.depth.createOutputQueue(maxSize=2, blocking=False), "depth"))
        else:
            print(f"Stereo pair {pair.left}/{pair.right} is not available as a camera output; skipping depth")
    elif args.no_stereo:
        print("--no-stereo set, skipping stereo depth")
    else:
        print("Device has no stereo pair, skipping stereo depth")

    if color_camera is not None and not args.no_nnet:
        archive = load_nn_archive(device, args.nn_model)
        color_output = cameras[color_camera.getBoardSocket()][1]
        frame_type = nn_input_type(device, archive)
        # The network takes frames at the rate of the sensor, for the same
        # reason as the encoders.
        for index in range(n_networks):
            # Only the first network gets the depth. The others are there to
            # keep the inference engines busy.
            first_with_depth = index == 0 and stereo is not None
            if index > 0:
                network = add_network(pipeline, color_output, archive, frame_type)
            elif first_with_depth:
                network = pipeline.create(dai.node.SpatialDetectionNetwork).build(
                    color_camera, stereo, archive, fps=camera_fps
                )
            else:
                network = pipeline.create(dai.node.DetectionNetwork).build(color_camera, archive, fps=camera_fps)

            if first_with_depth:
                network.setDepthLowerThreshold(100)
                network.setDepthUpperThreshold(5000)
                network.setBoundingBoxScaleFactor(0.5)

            limit_nn_shaves(network, archive, args.nn_shaves)
            network.setConfidenceThreshold(0.5)
            network.input.setBlocking(False)
            if index == 0:
                labels = network.getClasses()
                context.labels = list(labels) if labels else []
                streams.append(Stream("detections", network.out.createOutputQueue(maxSize=4, blocking=False), "detections"))
                context.detection_frame_name = f"preview_{color_camera.getBoardSocket().name}"
            else:
                streams.append(Stream(f"detections_{index}", network.out.createOutputQueue(maxSize=4, blocking=False), "extra"))
    elif color_camera is None:
        print("No color camera found, skipping neural network")
    else:
        print("--no-nnet set, skipping neural network")

    return pipeline, streams, control_queues, system_queue, context


def apply_ir(device: dai.Device, has_ir: bool, dot: Optional[float] = None, flood: Optional[float] = None) -> None:
    """Set the IR intensities, but only on a device that has the drivers.

    A device without IR LEDs reports a file descriptor error for every call.
    """
    if not has_ir:
        return
    if dot is not None:
        device.setIrLaserDotProjectorIntensity(dot)
    if flood is not None:
        device.setIrFloodLightIntensity(flood)


def send_manual_exposure(control_queues: list[Any], exposure: int, iso: int) -> None:
    control = dai.CameraControl()
    control.setManualExposure(exposure, iso)
    for queue in control_queues:
        queue.send(control)


def stress_test() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mxid", default="", help="Device ID, device name, or IP address")
    parser.add_argument("-ne", "--n-edge-detectors", default=0, type=int)
    parser.add_argument("--no-nnet", action="store_true", help="Do not create a detection network")
    parser.add_argument("--nn-model", default=NN_MODEL, help="Model zoo name of the detection model")
    parser.add_argument(
        "--nn-shaves",
        default=NN_SHAVES_RVC2,
        type=int,
        help="Number of shaves for the RVC2 blob. Decrease it if the device reports too few free shaves.",
    )
    parser.add_argument("--no-stereo", action="store_true", help="Do not create stereo depth")
    parser.add_argument("--camera-fps", type=float, help="Sensor frame rate. Higher values give more load.")
    parser.add_argument("--encoder-fps", type=float, help="Frame rate that the video encoders declare. Camera outputs always use --camera-fps.")
    parser.add_argument("--n-encoders", type=int, help="Number of video encoders per camera")
    parser.add_argument("--n-manips", type=int, help="Number of chained image manipulation nodes per camera")
    parser.add_argument("--n-nnets", type=int, help="Number of detection networks")
    parser.add_argument(
        "--full-res-color",
        dest="full_res_color",
        action="store_true",
        default=None,
        help="Use the largest sensor configuration for the color camera and downscale it on the device",
    )
    parser.add_argument("--no-full-res-color", dest="full_res_color", action="store_false", help="Do not use the largest sensor configuration")
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

    has_ir = bool(device.getIrDrivers())
    if not has_ir:
        print("Device has no IR drivers, skipping dot projector and flood light")

    if args.slow_rampup:
        apply_ir(device, has_ir, dot=0.0, flood=0.0)
    else:
        apply_ir(device, has_ir, dot=dot_intensity, flood=flood_intensity)

    start_time = time.monotonic()
    ramp_start = start_time if args.slow_rampup and has_ir else None
    last_ramp_update = 0.0
    last_dot = None
    last_flood = None
    last_frames: dict[str, np.ndarray] = {}

    try:
        usb_speed = device.getUsbSpeed()
        connection = f" over USB {usb_speed.name}" if usb_speed != dai.UsbSpeed.UNKNOWN else ""
        print(f"Started on {device.getPlatformAsString()}{connection}")
        while pipeline.isRunning():
            if ramp_start is not None:
                elapsed = time.monotonic() - ramp_start
                fraction = 1.0 if args.rampup_seconds <= 0 else clamp(elapsed / args.rampup_seconds, 0.0, 1.0)
                now = time.monotonic()
                if now - last_ramp_update >= 0.05 or fraction >= 1.0:
                    dot = dot_intensity * fraction
                    flood = flood_intensity * fraction
                    if last_dot is None or abs(dot - last_dot) >= 1e-3:
                        apply_ir(device, has_ir, dot=dot)
                        last_dot = dot
                    if last_flood is None or abs(flood - last_flood) >= 1e-3:
                        apply_ir(device, has_ir, flood=flood)
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
                        add_detection_overlay(frame, packet, context.labels)

            system_info = system_queue.tryGet()
            if system_info is not None:
                print(f"[{int(time.monotonic() - start_time)}s]{connection}")
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
                apply_ir(device, has_ir, dot=dot_intensity)
            elif key == ord("d"):
                dot_intensity = clamp(dot_intensity + DOT_STEP, 0.0, 1.0)
                apply_ir(device, has_ir, dot=dot_intensity)
            elif key == ord("s"):
                flood_intensity = clamp(flood_intensity - FLOOD_STEP, 0.0, 1.0)
                apply_ir(device, has_ir, flood=flood_intensity)
            elif key == ord("w"):
                flood_intensity = clamp(flood_intensity + FLOOD_STEP, 0.0, 1.0)
                apply_ir(device, has_ir, flood=flood_intensity)
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
