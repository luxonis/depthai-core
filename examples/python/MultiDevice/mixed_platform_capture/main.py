#!/usr/bin/env python3
"""An OAK4 detects a fist and triggers one capture on each RVC2 camera, in one multi-device pipeline.

The OAK4 runs hand-pose detection on its camera (or on a replayed video). Each RVC2 runs a Camera and a Script gate
that forwards the newest frame only when a trigger arrives. The visualizer shows the OAK4 video and
the last batch of captures until the next fist.
"""
import argparse
import time
from pathlib import Path

import depthai as dai
from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork

from utils.annotation_node import AnnotationNode
from utils.capture_gallery import CaptureGallery
from utils.process import ProcessDetections

MODELS = Path(__file__).parent / "depthai_models"
PADDING = 0.1
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
    parser.add_argument("--fps", type=float, default=15, help="Camera or replay FPS on the OAK4.")
    parser.add_argument("--capture-size", type=int, nargs=2, default=(1280, 800), metavar=("W", "H"))
    parser.add_argument("--save-dir", type=Path, help="Also write each batch as a JPEG to this folder.")
    parser.add_argument("--max-batches", type=int, default=0, help="Stop after this many batches (0 = run until 'q').")
    parser.add_argument("--ws-port", type=int, default=8765, help="Visualizer WebSocket port.")
    return parser.parse_args()


def open_oak4(name, attempts=20):
    # Right after an oakapp container build, the gate can hold the local device for some more seconds.
    for attempt in range(attempts):
        try:
            device = dai.Device(dai.DeviceInfo(name)) if name else dai.Device()
            break
        except RuntimeError as error:
            if "in use" not in str(error) or attempt == attempts - 1:
                raise
            print(f"OAK4 in use, retrying: {error}")
            time.sleep(3)
    if device.getPlatform() != dai.Platform.RVC4:
        raise RuntimeError(f"--oak4 must be an RVC4 device, {device.getDeviceId()} is {device.getPlatform().name}")
    return device


def create_fist_detector(pipeline, media, fps):
    """Hand-pose detection on the OAK4, as in the oak-examples hand-pose example."""
    palm_archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription.fromYamlFile("mediapipe_palm_detection.RVC4.yaml", str(MODELS))))
    hand_archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription.fromYamlFile("mediapipe_hand_landmarker.RVC4.yaml", str(MODELS))))

    if media:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(media)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
        replay.setLoop(True)
        replay.setFps(fps)
        frames = replay.out
    else:
        frames = pipeline.create(dai.node.Camera).build().requestOutput((768, 768), dai.ImgFrame.Type.BGR888i, fps=fps)

    resize = pipeline.create(dai.node.ImageManip)
    resize.setMaxOutputFrameSize(palm_archive.getInputWidth() * palm_archive.getInputHeight() * 3)
    resize.initialConfig.setOutputSize(palm_archive.getInputWidth(), palm_archive.getInputHeight(), mode=dai.ImageManipConfig.ResizeMode.STRETCH)
    resize.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888i)
    frames.link(resize.inputImage)

    hand_size = (hand_archive.getInputWidth(), hand_archive.getInputHeight())
    palm_nn = pipeline.create(ParsingNeuralNetwork).build(resize.out, palm_archive)
    crop_configs = pipeline.create(ProcessDetections).build(detections_input=palm_nn.out, padding=PADDING, target_size=hand_size)
    hand_crop = (
        pipeline.create(FrameCropper)
        .fromManipConfigs(inputManipConfigs=crop_configs.config_output, maxOutputFrameSize=hand_size[0] * hand_size[1] * 3, waitForConfig=True)
        .build(palm_nn.passthrough)
    )
    hand_nn = pipeline.create(ParsingNeuralNetwork).build(hand_crop.out, hand_archive)
    hands = pipeline.create(GatherData).build(cameraFps=fps, inputData=hand_nn.outputs, inputReference=palm_nn.out)
    annotations = pipeline.create(AnnotationNode).build(
        gathered_data=hands.out,
        video=frames,
        padding_factor=PADDING,
        confidence_threshold=CONFIDENCE_THRESHOLD,
        connections_pairs=hand_archive.getConfig().model.heads[0].metadata.extraParams["skeleton_edges"],
    )
    return frames, annotations


def create_video_stream(pipeline, frames, fps):
    to_nv12 = pipeline.create(dai.node.ImageManip)
    to_nv12.setMaxOutputFrameSize(768 * 768 * 3)
    to_nv12.initialConfig.setOutputSize(768, 768)
    to_nv12.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
    frames.link(to_nv12.inputImage)
    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setMaxOutputFrameSize(768 * 768 * 3)
    encoder.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H264_MAIN)
    to_nv12.out.link(encoder.input)
    return encoder.out


def add_capture_camera(pipeline, name, trigger, gallery, capture_size):
    device = pipeline.addDevice(name)
    print(f"Capture camera {device.getDeviceId()} ({device.getPlatform().name})")
    camera = pipeline.create(dai.node.Camera, device).build()
    gate = pipeline.create(dai.node.Script, device)
    gate.setScript(CAPTURE_GATE)
    gate.inputs["frame"].setBlocking(False)
    gate.inputs["frame"].setMaxSize(1)
    camera.requestOutput(capture_size, fps=CAPTURE_FPS).link(gate.inputs["frame"])
    trigger.link(gate.inputs["trigger"])
    gate.outputs["capture"].link(gallery.add_capture(f"{device.getDeviceId()} {device.getPlatform().name}"))


def main():
    args = parse_args()
    visualizer = dai.RemoteConnection(webSocketPort=args.ws_port, httpPort=8082)
    oak4 = open_oak4(args.oak4)
    print(f"OAK4 {oak4.getDeviceId()}")

    with dai.Pipeline(oak4) as pipeline:
        frames, annotations = create_fist_detector(pipeline, args.media, args.fps)

        gallery = pipeline.create(CaptureGallery)
        gallery.save_dir = args.save_dir
        if args.save_dir:
            args.save_dir.mkdir(parents=True, exist_ok=True)
        for name in args.rvc2:
            add_capture_camera(pipeline, name, annotations.out_fist, gallery, tuple(args.capture_size))

        visualizer.addTopic("Video", create_video_stream(pipeline, frames, args.fps), "images")
        visualizer.addTopic("Detections", annotations.out_detections, "images")
        visualizer.addTopic("Pose", annotations.out_pose_annotations, "images")
        visualizer.addTopic("Captures", gallery.out, "captures")

        pipeline.start()
        visualizer.registerPipeline(pipeline)
        print("Pipeline started. Show a fist to the OAK4 to capture on the other cameras.")

        while pipeline.isRunning():
            if visualizer.waitKey(1) == ord("q"):
                break
            if args.max_batches and gallery.batch_count >= args.max_batches:
                break


if __name__ == "__main__":
    main()
