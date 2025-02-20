#!/usr/bin/env python3
import cv2
import depthai as dai


# This script demonstrates how to use a warp mesh in DepthAI to transform
# incoming camera frames. A warp mesh is a grid of points where each point
# indicates from which location in the source image we sample pixels to place
# in the corresponding location of the output image.

# 3x3 WARP MESH (SAMPLE SHIFTS):
# ------------------------------
# Conceptually, we have 9 points arranged like this:

#     p0 ----- p1 ----- p2
#       |       |       |
#     p3 ----- p4 ----- p5
#       |       |       |
#     p6 ----- p7 ----- p8

# Each point is defined by an (x, y) coordinate in the source image.
# If all points were placed in their "natural" positions, we'd get
# an identity (no distortion) mapping. By shifting one or more points,
# we create warping or perspective effects.

# For example, if p3 is shifted horizontally, then at that grid
# position, we "pull" pixels from a different horizontal location,
# causing a horizontal distortion across that row.

# Below, we show how to create a slightly shifted 3x3 mesh.
# Feel free to adjust the coordinates to see how the output changes.


# Create pipeline
pipeline = dai.Pipeline()

width, height = 1280, 800
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

platform = pipeline.getDefaultDevice().getPlatform()
imgType = dai.ImgFrame.Type.BGR888p if platform == dai.Platform.RVC2 else dai.ImgFrame.Type.NV12

cameraOutput = camRgb.requestOutput((width, height), type=imgType)
originalFrameQueue = cameraOutput.createOutputQueue()

# Define a 3x3 warp mesh with a small horizontal shift towards the center the middle row
# and a small vertical shift towar
# Each point tells the warp node from which source coordinate
# to sample pixels for the final output.
p0_3x3 = dai.Point2f(0, 0)
p1_3x3 = dai.Point2f(width / 2, 200) # Identity would be (width / 2, 0)
p2_3x3 = dai.Point2f(width, 0)

p3_3x3 = dai.Point2f(300, height / 2) # Identity would be (0, height / 2)
p4_3x3 = dai.Point2f(width / 2, height / 2)
p5_3x3 = dai.Point2f(width - 300, height / 2) # Identity would be (width, height / 2)

p6_3x3 = dai.Point2f(0, height)
p7_3x3 = dai.Point2f(width / 2, height - 200) # Identity would be (width / 2, height)
p8_3x3 = dai.Point2f(width, height)

# Create and configure the Warp node
warp = pipeline.create(dai.node.Warp)
warp.setWarpMesh([
    p0_3x3, p1_3x3, p2_3x3,
    p3_3x3, p4_3x3, p5_3x3,
    p6_3x3, p7_3x3, p8_3x3
], 3, 3)

# Set output size and frame limits
warpOutputSize = (640, 480)
warp.setOutputSize(warpOutputSize)
warp.setMaxOutputFrameSize(warpOutputSize[0] * warpOutputSize[1] * 3)
warp.setInterpolation(dai.Interpolation.BILINEAR)

cameraOutput.link(warp.inputImage)
warpQueue = warp.out.createOutputQueue()

# Start the pipeline
pipeline.start()
with pipeline:
    while True:
        # Get and show original frame
        originalFrame = originalFrameQueue.get()
        assert isinstance(originalFrame, dai.ImgFrame)
        cv2.imshow("Original", originalFrame.getCvFrame())

        # Get and show warped frame
        warpedFrame = warpQueue.get()
        assert isinstance(warpedFrame, dai.ImgFrame)
        if platform == dai.Platform.RVC4:
            warpedFrame.setType(dai.ImgFrame.Type.GRAY8) # Chroma plane warping is not yet supported on RVC4
        cv2.imshow("Warped", warpedFrame.getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break
