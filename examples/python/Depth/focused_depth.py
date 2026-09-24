#!/usr/bin/env python3
"""Focused depth on RVC4: a fixed ROI or the largest detected object.

Open the printed Visualizer URL. Depth is in the rectified left-camera frame;
color/detections are separate views. --fps requests a camera rate, not a guaranteed
inference rate. The console reports measured depth output FPS after warm-up.
--depth-mode roi leaves the background empty; hold bridges --hold-frames empty
messages using fresh stereo frames; hybrid fills the background with downscaled
EVA stereo and overwrites the selected ROI with neural depth. Hybrid does not hold
missing detections. Neither mode predicts object motion or bridges a stalled detector.
"""

import argparse
import time
from urllib.parse import quote

import depthai as dai
import numpy as np


def parseRoi(text):
    try:
        values = tuple(float(value) for value in text.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError("ROI must contain four numbers") from error
    if len(values) != 4 or not (0 <= values[0] < values[2] <= 1 and 0 <= values[1] < values[3] <= 1):
        raise argparse.ArgumentTypeError("ROI must be normalized xmin,ymin,xmax,ymax")
    return values


def main(defaultMode="detector", budget=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("roi", "detector"), default=defaultMode)
    parser.add_argument("--depth-mode", choices=("roi", "hold", "hybrid"), default="roi",
                        help="ROI only; hold ROI through empty detections; or full EVA depth enhanced in the ROI")
    parser.add_argument("--hold-frames", type=int, default=2, help="Consecutive empty detections to bridge in hold mode")
    parser.add_argument("--stereo-size", type=int, nargs=2, default=(384, 240), metavar=("WIDTH", "HEIGHT"),
                        help="Downscaled EVA input size in hybrid mode (default: 384 240)")
    parser.add_argument("--roi", type=parseRoi, default=parseRoi("0.35,0.30,0.65,0.70"))
    parser.add_argument("--model", default="yolov6-nano", help="Object detection model")
    parser.add_argument("--depth-model", choices=("192X120", "288X180", "384X240", "480X300", "576X360"), default="576X360")
    parser.add_argument("--confidence", type=float, default=0.5, help="Detection confidence threshold")
    parser.add_argument("--fps", type=float, default=30.0, help="Requested camera FPS")
    parser.add_argument("--frames", type=int, default=0, help="Stop after N depth frames, including empty frames")
    parser.add_argument("--seconds", type=float, default=0, help="Stop after N seconds (including warm-up)")
    parser.add_argument("--headless", action="store_true", help="Measure without the browser visualizer")
    parser.add_argument("--debug", action="store_true", help="Print per-crop timing")
    parser.add_argument("--webSocketPort", type=int, default=8765)
    parser.add_argument("--httpPort", type=int, default=8082)
    args = parser.parse_args()
    if not np.isfinite(args.fps) or args.fps <= 0 or args.frames < 0 or not np.isfinite(args.seconds) or args.seconds < 0:
        parser.error("FPS must be positive; frames and seconds must be nonnegative")

    if args.hold_frames < 0:
        parser.error("Hold frames must be nonnegative")
    width, height = args.stereo_size
    if width <= 0 or height <= 0 or width % 128 or width > 1280 or height > 800:
        parser.error("Stereo size must be positive, width divisible by 128, and at most 1280x800")

    if not 0 <= args.confidence <= 1:
        parser.error("Confidence must be between zero and one")

    remote = None if args.headless else dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        if device.getPlatform() != dai.Platform.RVC4 or not device.isNeuralDepthSupported():
            raise RuntimeError("This example requires an RVC4 with NeuralDepth support")
        depth = pipeline.create(dai.node.Depth)
        depth.setFocusMode(getattr(dai.node.Depth.FocusMode, args.depth_mode.upper()))
        depth.setFocusHoldFrames(args.hold_frames)
        depth.setFocusStereoSize(*args.stereo_size)
        depth.setFocusModels([getattr(dai.DeviceModelZoo, "NEURAL_DEPTH_" + args.depth_model)])
        depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.ALL if budget else dai.node.Depth.FocusSelectionMode.LARGEST)
        depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.TIME_BUDGET if budget else dai.node.Depth.FocusDispatchMode.SINGLE_TIER_PER_FRAME)
        depth.build(args.fps)
        if args.mode == "detector":
            camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=args.fps)
            detector = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
            detector.setConfidenceThreshold(args.confidence)
            detector.out.link(depth.inputDetections)
            if remote:
                # Keep the RGB preview small so it does not consume the depth transport budget.
                color = camera.requestOutput((320, 200), type=dai.ImgFrame.Type.RGB888i, fps=args.fps)
                remote.addTopic("color", color, "color")
                remote.addTopic("detections", detector.out, "color")
        else:
            # Timestamp the fixed ROI from the camera, avoiding an unsynchronized host timer.
            left = pipeline.create(dai.node.Camera).build(device.getStereoPairs()[0].left, sensorFps=args.fps)
            trigger = left.requestOutput((64, 40), type=dai.ImgFrame.Type.GRAY8, fps=args.fps)
            script = pipeline.create(dai.node.Script)
            trigger.link(script.inputs["frame"])
            script.setScript(f"""
while True:
    frame = node.io["frame"].get()
    detection = ImgDetection()
    detection.xmin, detection.ymin, detection.xmax, detection.ymax = {args.roi!r}
    detection.confidence = 1.0
    message = ImgDetections()
    message.detections = [detection]
    message.setTimestamp(frame.getTimestamp())
    message.setTimestampDevice(frame.getTimestampDevice())
    message.setSequenceNum(frame.getSequenceNum())
    node.io["detections"].send(message)
""")
            script.outputs["detections"].link(depth.inputDetections)

        # Link detections before accessing the lazy focused output.
        output = depth.focusedDepth
        depthQueue = output.createOutputQueue(maxSize=4, blocking=False)
        debugQueue = depth.focusDebug.createOutputQueue(maxSize=4, blocking=False)
        if remote:
            remote.addTopic("focused depth (left camera)", output, "depth")

        pipeline.start()
        if remote:
            remote.registerPipeline(pipeline)
            wsUrl = quote(f"ws://localhost:{args.webSocketPort}", safe="")
            print(f"Visualizer: http://localhost:{args.httpPort}?ws_url={wsUrl}", flush=True)
        print(f"Mode={args.mode}/{args.depth_mode}, depth model={args.depth_model}, requested camera FPS={args.fps:g}", flush=True)
        if budget:
            print("Time budget is best effort; one crop may exceed the frame period.", flush=True)
        start = time.monotonic()
        first = last = None
        frames = measured = nonempty = 0
        lastReport = start
        try:
            while pipeline.isRunning():
                now = time.monotonic()
                if args.seconds and now - start >= args.seconds:
                    break
                if remote and remote.waitKey(1) == ord("q"):
                    break
                message = depthQueue.tryGet()
                if message is not None:
                    frames += 1
                    if now - start >= 5:
                        first = now if first is None else first
                        last = now
                        measured += 1
                        nonempty += bool(np.any(message.getFrame()))
                for debug in debugQueue.tryGetAll():
                    if args.debug:
                        print(bytes(debug.getData()).decode("utf-8", "replace"), flush=True)
                if now - lastReport >= 2 and measured >= 30:
                    print(f"Depth output: {(measured - 1) / (last - first):.2f} FPS; nonempty={nonempty}/{measured}", flush=True)
                    lastReport = now
                if args.frames and frames >= args.frames:
                    break
                time.sleep(0.001)
        except KeyboardInterrupt:
            pass
        finally:
            if measured > 1:
                print(f"RESULT: {(measured - 1) / (last - first):.2f} depth FPS; {nonempty}/{measured} nonempty frames after warm-up", flush=True)


if __name__ == "__main__":
    main()
