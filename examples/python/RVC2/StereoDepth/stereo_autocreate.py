#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import time

class StereoVisualizer(dai.node.HostNode):
    def process(self, inFrame: dai.ImgFrame):
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
    # Allow stereo inputs to be created automatically
    # NOTE: This is a naive implementation, it will not handle correctly the case where cameras have already been created
    stereo = pipeline.create(dai.node.StereoDepth).build(autoCreateCameras=True)
    visualizer = pipeline.create(StereoVisualizer)
    visualizer.link_args(stereo.disparity)
    visualizer.sendProcessingToPipeline(True)

    pipeline.run()
