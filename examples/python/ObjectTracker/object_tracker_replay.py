#!/usr/bin/env python3

import cv2
import depthai as dai
import time

from pathlib import Path
from argparse import ArgumentParser

scriptDir = Path(__file__).resolve().parent
examplesRoot = (scriptDir / Path('../')).resolve()  # This resolves the parent directory correctly
models = examplesRoot / 'models'
videoPath = models / 'construction_vest.mp4'

parser = ArgumentParser()
parser.add_argument("-i", "--inputVideo", default=videoPath, help="Input video name")
parser.add_argument("-c", "--camera", type=bool, help="Use camera as input", default=False)
args = parser.parse_args()

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    inputSource = None
    if args.camera: 
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        inputSource = camRgb
    else:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(args.inputVideo))
        inputSource = replay

    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(inputSource, "yolov6-nano")
    objectTracker = pipeline.create(dai.node.ObjectTracker)

    detectionNetwork.setConfidenceThreshold(0.6)
    detectionNetwork.input.setBlocking(False)
    labelMap = detectionNetwork.getClasses()

    objectTracker.setDetectionLabelsToTrack([0])  # track only person
    # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
    # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

    preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
    tracklets = objectTracker.out.createOutputQueue()

    detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

    detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    detectionNetwork.out.link(objectTracker.inputDetections)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    pipeline.start()
    while(pipeline.isRunning()):
        imgFrame = preview.get()
        track = tracklets.get()
        assert isinstance(imgFrame, dai.ImgFrame), "Expected ImgFrame"
        assert isinstance(track, dai.Tracklets), "Expected Tracklets"

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        for t in trackletsData:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)

            try:
                label = labelMap[t.label]
            except:
                label = t.label

            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

        cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == ord('q'):
            break
