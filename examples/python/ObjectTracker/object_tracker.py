#!/usr/bin/env python3

import cv2
import depthai as dai
import time


fullFrameTracking = False
useSpatialAssociation = False
fps = 20

with dai.Pipeline() as pipeline:
    colorSocket = dai.CameraBoardSocket.CAM_A
    for features in pipeline.getDefaultDevice().getConnectedCameraFeatures():
        if dai.CameraSensorType.COLOR in features.supportedTypes:
            colorSocket = features.socket
            break
    camRgb = pipeline.create(dai.node.Camera).build(colorSocket, sensorFps=fps)

    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, fps, (640, 400))

    spatialDetectionNetwork = pipeline.create(dai.node.SpatialDetectionNetwork).build(camRgb, depth, "yolov6-nano")
    objectTracker = pipeline.create(dai.node.ObjectTracker)

    spatialDetectionNetwork.setConfidenceThreshold(0.6)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)
    labelMap = spatialDetectionNetwork.getClasses()

    objectTracker.setDetectionLabelsToTrack([0])
    objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
    objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
    if useSpatialAssociation:
        objectTracker.setSpatialAssociation(True)
        objectTracker.setSpatialAssociationWeight(0.5)
        objectTracker.setSpatialDistanceThreshold(1.5)
        objectTracker.setSpatialDepthAwareScale(0.1)

    preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
    tracklets = objectTracker.out.createOutputQueue()

    if fullFrameTracking:
        camRgb.requestFullResolutionOutput().link(objectTracker.inputTrackerFrame)
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
            if t.velocity is not None and t.speed is not None:
                cv2.putText(frame, f"Velocity X: {t.velocity.x:.2f} m/s", (x1 + 10, y1 + 110), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Velocity Y: {t.velocity.y:.2f} m/s", (x1 + 10, y1 + 125), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Velocity Z: {t.velocity.z:.2f} m/s", (x1 + 10, y1 + 140), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Speed: {t.speed:.2f} m/s", (x1 + 10, y1 + 155), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

        cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == ord('q'):
            break
