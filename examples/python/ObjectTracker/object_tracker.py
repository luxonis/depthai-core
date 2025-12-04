#!/usr/bin/env python3

import cv2
import depthai as dai
import time


fullFrameTracking = False

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define sources and outputs
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    leftOutput = monoLeft.requestOutput((640, 400))
    rightOutput = monoRight.requestOutput((640, 400))
    leftOutput.link(stereo.left)
    rightOutput.link(stereo.right)

    spatialDetectionNetwork = pipeline.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, "yolov6-nano")
    objectTracker = pipeline.create(dai.node.ObjectTracker)

    spatialDetectionNetwork.setConfidenceThreshold(0.6)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)
    labelMap = spatialDetectionNetwork.getClasses()

    objectTracker.setDetectionLabelsToTrack([0])  # track only person
    # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
    # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

    preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
    tracklets = objectTracker.out.createOutputQueue()

    if fullFrameTracking:
        camRgb.requestFullResolutionOutput().link(objectTracker.inputTrackerFrame)
        # do not block the pipeline if it's too slow on full frame
        objectTracker.inputTrackerFrame.setBlocking(False)
        objectTracker.inputTrackerFrame.setMaxSize(1)
    else:
        spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

    spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    spatialDetectionNetwork.out.link(objectTracker.inputDetections)

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

            cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

        cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == ord('q'):
            break
