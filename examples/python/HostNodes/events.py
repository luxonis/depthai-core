
#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

# Get argument first
modelDescription = dai.NNModelDescription(modelSlug="yolov6-nano", platform="RVC2")
archivePath = dai.getModelFromZoo(modelDescription, useCached=True)

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    # Properties
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(15)
    nnArchive = dai.NNArchive(archivePath)
    h, w = nnArchive.getConfig().getConfigV1().model.inputs[0].shape[-2:]
    camRgb.setPreviewSize(w, h)
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        camRgb.preview, nnArchive
    )
    detectionNetwork.setNumInferenceThreads(2)

    # If needed, you can set the NNArchive by yourself
    # detectionNetwork.setNNArchive(nnArchive)

    # If nnArchive.getModelType() == dai.ModelType.SUPERBLOB
    # you can specify the number of shaves
    # detectionNetwork.setNNArchive(nnArchive, numShaves=9)
    # When ^^^ is used and the archive type is not SUPERBLOB, an exception will be thrown

    qRgb = detectionNetwork.passthrough.createOutputQueue()
    qDet = detectionNetwork.out.createOutputQueue()

    labelMap = detectionNetwork.getClasses()
    eventMan = dai.EventsManager()
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

    eventSent = False
    while pipeline.isRunning():
        inRgb: dai.ImgFrame = qRgb.get()
        inDet: dai.ImgDetections = qDet.get()
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            print(eventMan.sendEvent("rgb", inRgb, ["dets"]))
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

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
