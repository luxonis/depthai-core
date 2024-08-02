#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time

USE_REPLAY = False

examplesRoot = Path(__file__).parent / Path('../../').resolve()
models = examplesRoot / Path('models')
videoPath = models / Path('construction_vest.mp4')

# Download yolo model from zoo
modelDescription = dai.NNModelDescription(modelSlug="yolov10-nano", platform="RVC4")
modelPath = dai.getModelFromZoo(modelDescription, useCached=True)
# Create pipeline
with dai.Pipeline() as pipeline:

    # Define sources and outputs
    if USE_REPLAY:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(videoPath)
        replay.setSize(512, 288)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
        sourceOutput = replay.out
    else:
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        sourceOutput = camRgb.requestOutput((512, 288), dai.ImgFrame.Type.BGR888i)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork)
    detectionNetwork.setNNArchive(dai.NNArchive(modelPath))
    sourceOutput.link(detectionNetwork.input)
    labelMap = detectionNetwork.getClasses()

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()

    pipeline.start()

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(name, frame):
        color = (255, 0, 0)
        for detection in detections:
            bbox = frameNorm(
                frame,
                (detection.xmin, detection.ymin, detection.xmax, detection.ymax),
            )
            cv2.putText(
                frame,
                labelMap[detection.label],
                (bbox[0] + 10, bbox[1] + 20),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.putText(
                frame,
                f"{int(detection.confidence * 100)}%",
                (bbox[0] + 10, bbox[1] + 40),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                255,
            )
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        # Show the frame
        cv2.imshow(name, frame)

    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            cv2.putText(
                frame,
                "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                (2, frame.shape[0] - 4),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.4,
                color2,
            )

        if inDet is not None:
            detections = inDet.detections
            counter += 1

        if frame is not None:
            displayFrame("rgb", frame)
            print("FPS: {:.2f}".format(counter / (time.monotonic() - startTime)))
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
