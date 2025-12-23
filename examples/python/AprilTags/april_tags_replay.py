#!/usr/bin/env python3

import cv2
import depthai as dai
import time
from pathlib import Path

# Get the absolute path of the current script's directory
script_dir = Path(__file__).resolve().parent
examplesRoot = (script_dir / Path('../')).resolve()  # This resolves the parent directory correctly
models = examplesRoot / 'models'
tagImage = models / 'april_tags.jpg'

class ImageReplay(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output =  self.createOutput()
        frame = cv2.imread(str(tagImage.resolve()))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imgFrame = dai.ImgFrame()
        imgFrame.setData(frame)
        imgFrame.setWidth(frame.shape[1])
        imgFrame.setHeight(frame.shape[0])
        imgFrame.setType(dai.ImgFrame.Type.GRAY8)
        self.imgFrame = imgFrame
    def run(self):
        while self.mainLoop():
            self.output.send(self.imgFrame)
            time.sleep(0.03)

with dai.Pipeline() as pipeline:
    imageReplay = pipeline.create(ImageReplay)
    aprilTagNode = pipeline.create(dai.node.AprilTag)
    imageReplay.output.link(aprilTagNode.inputImage)
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
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        def to_int(tag):
            return (int(tag.x), int(tag.y))

        for tag in aprilTags:
            topLeft = to_int(tag.topLeft)
            topRight = to_int(tag.topRight)
            bottomRight = to_int(tag.bottomRight)
            bottomLeft = to_int(tag.bottomLeft)

            center = (int((topLeft[0] + bottomRight[0]) / 2), int((topLeft[1] + bottomRight[1]) / 2))

            cv2.line(frame, topLeft, topRight, color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, topRight,bottomRight, color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, bottomRight,bottomLeft, color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, bottomLeft,topLeft, color, 2, cv2.LINE_AA, 0)

            idStr = "ID: " + str(tag.id)
            cv2.putText(frame, idStr, center, cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            cv2.putText(frame, f"fps: {fps:.1f}", (200, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

        cv2.imshow("detections", frame)
        if cv2.waitKey(1) == ord("q"):
            break
