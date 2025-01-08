#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np

# Create pipeline
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setPreviewSize(496, 496)
camRgb.setInterleaved(False)
maxFrameSize = camRgb.getPreviewWidth() * camRgb.getPreviewHeight() * 3

# Warp preview frame 1
warp1 = pipeline.create(dai.node.Warp)
# Create a custom warp mesh
tl = dai.Point2f(20, 20)
tr = dai.Point2f(460, 20)
ml = dai.Point2f(100, 250)
mr = dai.Point2f(400, 250)
bl = dai.Point2f(20, 460)
br = dai.Point2f(460, 460)
warp1.setWarpMesh([tl,tr,ml,mr,bl,br], 2, 3)
WARP1_OUTPUT_FRAME_SIZE = (992,500)
warp1.setOutputSize(WARP1_OUTPUT_FRAME_SIZE)
warp1.setMaxOutputFrameSize(WARP1_OUTPUT_FRAME_SIZE[0] * WARP1_OUTPUT_FRAME_SIZE[1] * 3)
warp1.setHwIds([1])
warp1.setInterpolation(dai.Interpolation.NEAREST_NEIGHBOR)

camRgb.preview.link(warp1.inputImage)
outQueue1 = warp1.out.createOutputQueue()

# Warp preview frame 2
warp2 = pipeline.create(dai.node.Warp)
# Create a custom warp mesh
mesh2 = [
    (20, 20), (250, 100), (460, 20),
    (100, 250), (250, 250), (400, 250),
    (20, 480), (250, 400), (460,480)
]
warp2.setWarpMesh(mesh2, 3, 3)
warp2.setMaxOutputFrameSize(maxFrameSize)
warp1.setHwIds([2])
warp2.setInterpolation(dai.Interpolation.BICUBIC)

camRgb.preview.link(warp2.inputImage)
outQueue2 = warp2.out.createOutputQueue()

pipeline.start()
with pipeline:
    while True:
        in1 = outQueue1.get()
        in2 = outQueue2.get()
        cv2.imshow("out1", in1.getCvFrame())
        cv2.imshow("out2", in2.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
