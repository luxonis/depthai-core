#!/usr/bin/env python3

import cv2
import depthai as dai
import time

with dai.Pipeline() as pipeline:
    hostCamera = pipeline.create(dai.node.Camera).build()
    aprilTagNode = pipeline.create(dai.node.AprilTag)
    hostCamera.requestOutput((1920, 1080)).link(aprilTagNode.inputImage)
    aprilTagNode.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_16H5)

    passthroughOutputQueue = aprilTagNode.passthroughInputImage.createOutputQueue()
    outQueue = aprilTagNode.out.createOutputQueue()

    color = (0, 255, 0)
    startTime = time.monotonic()
    counter = 0
    fps = 0.0

    pipeline.start()
    while pipeline.isRunning():
        aprilTagMessage = outQueue.get()
        assert(isinstance(aprilTagMessage, dai.AprilTags))
        aprilTags = aprilTagMessage.aprilTags

        counter += 1
        currentTime = time.monotonic()
        if (currentTime - startTime) > 1:
            fps = counter / (currentTime - startTime)
            counter = 0
            startTime = currentTime

        passthroughImage: dai.ImgFrame = passthroughOutputQueue.get()
        frame = passthroughImage.getCvFrame()
        for aprilTagDetection in aprilTags:
            topLeft = aprilTagDetection.topLeft
            topRight = aprilTagDetection.topRight
            bottomRight = aprilTagDetection.bottomRight
            bottomLeft = aprilTagDetection.bottomLeft

            center = (int((topLeft.x + bottomRight.x) / 2), int((topLeft.y + bottomRight.y) / 2))

            cv2.line(frame, (int(topLeft.x), int(topLeft.y)), (int(topRight.x), int(topRight.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(topRight.x), int(topRight.y)), (int(bottomRight.x), int(bottomRight.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(bottomRight.x), int(bottomRight.y)), (int(bottomLeft.x), int(bottomLeft.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(bottomLeft.x), int(bottomLeft.y)), (int(topLeft.x), int(topLeft.y)), color, 2, cv2.LINE_AA, 0)

            idStr = "ID: " + str(aprilTagDetection.id)
            cv2.putText(frame, idStr, center, cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            cv2.putText(frame, f"fps: {fps:.1f}", (200, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

        cv2.imshow("detections", frame)
        if cv2.waitKey(1) == ord("q"):
            break
