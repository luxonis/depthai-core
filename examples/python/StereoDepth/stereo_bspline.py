#!/usr/bin/env python3
"""Stereo depth rectified with the B-spline lens models of the device calibration (RVC4).

Uses BSPLINE rectification when the calibration has a B-spline lens model for both stereo cameras, otherwise the
default LEGACY (OpenCV) rectification. B-spline rectification runs at the native resolution of the lens models.
"""
import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    calibration = pipeline.getDefaultDevice().readCalibration()
    lenses = [calibration.getBsplineCameraCalibration(socket) for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)]
    useBspline = all(lens is not None for lens in lenses)
    size = (lenses[0].width, lenses[0].height) if useBspline else (640, 400)
    print("Rectification:", "BSPLINE" if useBspline else "LEGACY (no B-spline lens models in the calibration)", size)

    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth).build(left.requestOutput(size), right.requestOutput(size))
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
    if useBspline:
        stereo.setRectificationModel(dai.node.StereoDepth.RectificationModel.BSPLINE)
    depthQueue = stereo.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        depth = depthQueue.get()
        cv2.imshow("depth", dai.utility.colorizeDepthFrame(depth).getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
