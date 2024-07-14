#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
from enum import Enum

class Model(Enum):
    YOLO_V10 = 1
    YOLO_WORLD = 2
    YOLO_WORLD_MULTI = 3

USE_REPLAY = True
USE_MODE = Model.YOLO_WORLD_MULTI



def model_to_strings(argument):
    switcher = {
        Model.YOLO_V10: "yolov10n.dlc",
        Model.YOLO_WORLD: "yolo_world_v2_model_only.dlc",
        Model.YOLO_WORLD_MULTI: "yolo_world_v2_texts.dlc",
    }

    return switcher.get(argument, "nothing")

examplesRoot = Path(__file__).parent / Path('../../').resolve()
models = examplesRoot / Path('models')
videoPath = models / Path('construction_vest.mp4')
modelPath = models / Path(model_to_strings(USE_MODE))

# tiny yolo v4 label texts
labelMap = [
    "person",         "bicycle",    "car",           "motorbike",     "aeroplane",   "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",   "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",       "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",    "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball", "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",      "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",      "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",       "donut",         "cake",
    "chair",          "sofa",       "pottedplant",   "bed",           "diningtable", "toilet",        "tvmonitor",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",  "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",       "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"
]


# Create pipeline
info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3
with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:

        # Define sources and outputs
        if USE_REPLAY:
            replay = pipeline.create(dai.node.ReplayVideo)
            replay.setReplayVideoFile(videoPath)
            replay.setSize(640, 640)
            replay.setOutFrameType(dai.ImgFrame.Type.BGR888i)
            sourceOutput = replay.out
            
        else:
            camRgb = pipeline.create(dai.node.Camera)
            camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            sourceOutput = camRgb.requestOutput((640, 640), dai.ImgFrame.Type.BGR888i)
        nn = pipeline.create(dai.node.NeuralNetwork)
        nn.setNumInferenceThreads(1)
        nn.setXmlModelPath(modelPath, "/dev/null")
        nn.setBackend("snpe")
        sourceOutput.link(nn.input)

        detectionParser = pipeline.create(dai.node.DetectionParser)
        # Properties
        detectionParser.setConfidenceThreshold(0.25)
        detectionParser.setNumClasses(80)
        detectionParser.setCoordinateSize(4)
        detectionParser.setIouThreshold(0.25)
        detectionParser.setInputImageSize(640, 640)
        nn.out.link(detectionParser.input)


        qRgb = nn.passthrough.createOutputQueue()
        qDet = detectionParser.out.createOutputQueue()

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