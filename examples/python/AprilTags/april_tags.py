#!/usr/bin/env python3

import cv2
import depthai as dai
import time
from pathlib import Path

examplesRoot = Path(__file__).parent / Path('../').resolve()
models = examplesRoot / Path('models')
tagImage = models / Path('april_tags.jpg')

class HostCamera(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
        frame = cv2.imread(str(tagImage.resolve()))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imgFrame = dai.ImgFrame()
        imgFrame.setData(frame)
        imgFrame.setWidth(frame.shape[1])
        imgFrame.setHeight(frame.shape[0])
        imgFrame.setType(dai.ImgFrame.Type.GRAY8)
        self.imgFrame = imgFrame
    def run(self):
        while self.isRunning():
            self.output.send(self.imgFrame)
            time.sleep(0.005)

device = dai.Device()
with dai.Pipeline(device) as pipeline:
    print(device.getDeviceInfo())
    hostCamera = pipeline.create(HostCamera)
    aprilTag = pipeline.create(dai.node.AprilTag)
    hostCamera.output.link(aprilTag.inputImage)
    aprilTag.initialConfig.setFamily(dai.AprilTagConfig.Family.TAG_16H5)

    outputFeaturePassthroughQueue = aprilTag.passthroughInputImage.createOutputQueue()
    outQueue = aprilTag.out.createOutputQueue()

    color = (0, 255, 0)
    startTime = time.monotonic()
    counter = 0
    fps = 0

    pipeline.start()
    while pipeline.isRunning():
        aprilTagData = outQueue.get().aprilTags

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        outputPassthroughImage : dai.ImgFrame = outputFeaturePassthroughQueue.get()

        passthroughImage = outputPassthroughImage.getCvFrame()
        passthroughImage = cv2.cvtColor(passthroughImage, cv2.COLOR_GRAY2BGR)

        frame = passthroughImage.copy()

        for aprilTag in aprilTagData:
            topLeft = aprilTag.topLeft
            topRight = aprilTag.topRight
            bottomRight = aprilTag.bottomRight
            bottomLeft = aprilTag.bottomLeft

            center = (int((topLeft.x + bottomRight.x) / 2), int((topLeft.y + bottomRight.y) / 2))

            cv2.line(frame, (int(topLeft.x), int(topLeft.y)), (int(topRight.x), int(topRight.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(topRight.x), int(topRight.y)), (int(bottomRight.x), int(bottomRight.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(bottomRight.x), int(bottomRight.y)), (int(bottomLeft.x), int(bottomLeft.y)), color, 2, cv2.LINE_AA, 0)
            cv2.line(frame, (int(bottomLeft.x), int(bottomLeft.y)), (int(topLeft.x), int(topLeft.y)), color, 2, cv2.LINE_AA, 0)

            idStr = "ID: " + str(aprilTag.id)
            cv2.putText(frame, idStr, center, cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

            cv2.putText(frame, f"fps: {fps:.1f}", (200, 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)

        cv2.imshow("detections", frame)
        # cv2.imshow("passthrough", passthroughImage)

        if cv2.waitKey(1) == ord('q'):
            break