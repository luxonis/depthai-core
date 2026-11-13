#!/usr/bin/env python3
"""An OAK4 detects a fist and triggers one capture on each RVC2 camera, in one multi-device pipeline.

The visualizer shows one stream for the OAK4 (hand tracking) and one stream for each capture
camera. A capture stream is black until the first fist, then it shows the last capture until the
next fist. Each capture camera runs a Camera and a Script gate that forwards the newest frame only
when a trigger arrives, so only the captures go over the link.
"""
import argparse
import time
from pathlib import Path

import depthai as dai
from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork

from utils.annotation_node import AnnotationNode
from utils.capture_view import CaptureView
from utils.process import ProcessDetections

MODELS = Path(__file__).parent / "depthai_models"
CONFIDENCE_THRESHOLD = 0.5
CAPTURE_FPS = 5

CAPTURE_GATE = """
while True:
    frame = node.inputs["frame"].get()
    if node.inputs["trigger"].tryGet() is not None:
        node.io["capture"].send(frame)
"""


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--oak4", metavar="IP", help="OAK4 master. Default: the local device when the app runs on the OAK4.")
    parser.add_argument("--rvc2", nargs="+", required=True, metavar="IP", help="Capture cameras, for example two RVC2 PoE devices.")
    parser.add_argument("--media", type=Path, help="Replay this video with a fist into the OAK4 instead of its camera.")
    parser.add_argument("--size", type=int, default=1440, help="Side of the square OAK4 stream in pixels.")
    parser.add_argument("--fps", type=float, default=30, help="OAK4 camera FPS.")
    parser.add_argument("--bitrate", type=int, default=16000, help="OAK4 stream H.264 bitrate in kbps.")
    parser.add_argument("--capture-size", type=int, nargs=2, default=(1920, 1080), metavar=("W", "H"),
                        help="Capture size. A camera whose sensor is smaller uses its largest size.")
    parser.add_argument("--save-dir", type=Path, help="Also write each capture as a JPEG to this folder.")
    parser.add_argument("--max-captures", type=int, default=0, help="Stop after each camera has this many captures (0 = run until 'q').")
    parser.add_argument("--ws-port", type=int, default=8765, help="Visualizer WebSocket port.")
    return parser.parse_args()


def open_oak4(name, attempts=20):
    # A session that just ended (or the oakapp startup) can hold the device for some more seconds.
    for attempt in range(attempts):
        try:
            device = dai.Device(dai.DeviceInfo(name)) if name else dai.Device()
            break
        except (RuntimeError, ValueError) as error:
            if not any(text in str(error) for text in ("in use", "already used")) or attempt == attempts - 1:
                raise
            print(f"OAK4 in use, retrying: {error}")
            time.sleep(3)
    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError(f"--oak4 must be an RVC4 device, {device.getDeviceId()} is {device.getPlatform().name}")
    return device


def create_oak4_input(pipeline, media, size, fps):
    """Return BGR frames for the models and NV12 frames of the same view for the video stream."""
    if media:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(media)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
        replay.setLoop(True)
        replay.setFps(fps)
        to_nv12 = pipeline.create(dai.node.ImageManip)
        to_nv12.setMaxOutputFrameSize(size * size * 3)
        to_nv12.initialConfig.setOutputSize(size, size)
        to_nv12.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
        replay.out.link(to_nv12.inputImage)
        return replay.out, to_nv12.out
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    frames = camera.requestOutput((size, size), dai.ImgFrame.Type.BGR888i, fps=fps)
    video = camera.requestOutput((size, size), dai.ImgFrame.Type.NV12, fps=fps)
    return frames, video


def create_fist_detector(pipeline, frames, fps):
    """MediaPipe palm detection and hand landmarks on the OAK4."""
    palm_archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription.fromYamlFile("mediapipe_palm_detection.RVC4.yaml", str(MODELS))))
    hand_archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription.fromYamlFile("mediapipe_hand_landmarker.RVC4.yaml", str(MODELS))))

    palm_input = pipeline.create(dai.node.ImageManip)
    palm_input.setMaxOutputFrameSize(palm_archive.getInputWidth() * palm_archive.getInputHeight() * 3)
    palm_input.initialConfig.setOutputSize(palm_archive.getInputWidth(), palm_archive.getInputHeight(), mode=dai.ImageManipConfig.ResizeMode.STRETCH)
    palm_input.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888i)
    frames.link(palm_input.inputImage)
    palm_nn = pipeline.create(ParsingNeuralNetwork).build(palm_input.out, palm_archive)

    hand_size = (hand_archive.getInputWidth(), hand_archive.getInputHeight())
    crop_configs = pipeline.create(ProcessDetections).build(detections_input=palm_nn.out, target_size=hand_size)
    # Crop from the palm model input: it leaves the model together with the detections, so each crop config finds its frame.
    hand_crops = (
        pipeline.create(FrameCropper)
        .fromManipConfigs(inputManipConfigs=crop_configs.config_output, maxOutputFrameSize=hand_size[0] * hand_size[1] * 3, waitForConfig=True)
        .build(palm_nn.passthrough)
    )
    hand_nn = pipeline.create(ParsingNeuralNetwork).build(hand_crops.out, hand_archive)
    hands = pipeline.create(GatherData).build(cameraFps=fps, inputData=hand_nn.outputs, inputReference=palm_nn.out)
    return pipeline.create(AnnotationNode).build(
        gathered_data=hands.out,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        connections_pairs=hand_archive.getConfig().model.heads[0].metadata.extraParams["skeleton_edges"],
    )


def create_video_stream(pipeline, video, size, fps, bitrate_kbps):
    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setMaxOutputFrameSize(size * size * 3)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H264_HIGH)
    encoder.setBitrateKbps(bitrate_kbps)
    video.link(encoder.input)
    return encoder.out


def capture_size(device, wanted):
    features = {f.socket: f for f in device.getConnectedCameraFeatures()}
    largest = max(features[dai.CameraBoardSocket.CAM_A].configs, key=lambda c: c.width * c.height)
    if largest.width >= wanted[0] and largest.height >= wanted[1]:
        return tuple(wanted)
    return (largest.width, largest.height)


def add_capture_camera(pipeline, name, trigger, wanted_size, save_dir):
    device = pipeline.addDevice(name)
    size = capture_size(device, wanted_size)
    label = f"{device.getPlatform().name} {device.getDeviceId()}"
    print(f"Capture camera {label}, {size[0]}x{size[1]}")
    camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A)
    gate = pipeline.create(dai.node.Script, device)
    gate.setScript(CAPTURE_GATE)
    gate.inputs["frame"].setBlocking(False)
    gate.inputs["frame"].setMaxSize(1)
    camera.requestOutput(size, dai.ImgFrame.Type.NV12, fps=CAPTURE_FPS).link(gate.inputs["frame"])
    trigger.link(gate.inputs["trigger"])
    return label, pipeline.create(CaptureView).build(gate.outputs["capture"], size, device.getDeviceId(), save_dir)


def main():
    args = parse_args()
    visualizer = dai.RemoteConnection(webSocketPort=args.ws_port, httpPort=8082)
    oak4 = open_oak4(args.oak4)
    print(f"OAK4 {oak4.getDeviceId()}, {args.size}x{args.size} at {args.fps:g} FPS, {args.bitrate} kbps")
    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    with dai.Pipeline(oak4) as pipeline:
        frames, video = create_oak4_input(pipeline, args.media, args.size, args.fps)
        annotations = create_fist_detector(pipeline, frames, args.fps)
        visualizer.addTopic("OAK4", create_video_stream(pipeline, video, args.size, args.fps, args.bitrate), "OAK4")
        visualizer.addTopic("Hands", annotations.out_pose_annotations, "OAK4")

        views = []
        for name in args.rvc2:
            label, view = add_capture_camera(pipeline, name, annotations.out_fist, args.capture_size, args.save_dir)
            visualizer.addTopic(label, view.out, label)
            views.append(view)

        pipeline.start()
        visualizer.registerPipeline(pipeline)
        print("Pipeline started. Make a fist in front of the OAK4 to capture on the other cameras.")

        while pipeline.isRunning():
            if visualizer.waitKey(1) == ord("q"):
                break
            if args.max_captures and all(view.capture_count >= args.max_captures for view in views):
                break


if __name__ == "__main__":
    main()
