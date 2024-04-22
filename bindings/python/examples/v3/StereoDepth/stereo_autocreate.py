#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import time

class StereoVisualizer(dai.node.HostNode):
    def __init__(self):
        dai.node.HostNode.__init__(self)
        self.inputDepth = self.inputs["disparity"]

    def build(self, output: dai.Node.Output):
        output.link(self.inputDepth)
        return self

    def runOnce(self, messages: dai.MessageGroup):
        inFrame : dai.ImgFrame = messages["disparity"]
        outFrame = inFrame.getFrame()

        # Colorize the disparity map
        maxDisparity = np.max(outFrame)
        outFrame = (outFrame * (255 / maxDisparity)).astype(np.uint8)
        outFrame = cv2.applyColorMap(outFrame, cv2.COLORMAP_JET)

        # Normalization for better visualization
        cv2.imshow("depth", outFrame)
        if cv2.waitKey(1) == ord('q'):
            pipeline.stop()


# Create pipeline
with dai.Pipeline() as pipeline:
    # Let stereo depth output autocreate
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.build(autoCreateCameras=True)
    visualizer = pipeline.create(StereoVisualizer).build(stereo.disparity)
    pipeline.start()
    while pipeline.isRunning():
        time.sleep(0.1)