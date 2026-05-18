#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np
import time

pipeline = dai.Pipeline()
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
stereo = pipeline.create(dai.node.StereoDepth)

# Linking
monoLeftOut = monoLeft.requestFullResolutionOutput()
monoRightOut = monoRight.requestFullResolutionOutput()
monoLeftOut.link(stereo.left)
monoRightOut.link(stereo.right)

stereo.setRectification(True)
stereo.setExtendedDisparity(True)
stereo.setLeftRightCheck(True)

disparityQueue = stereo.disparity.createOutputQueue()
confidenceQueue = stereo.confidenceMap.createOutputQueue()

colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

with pipeline:
    pipeline.start()
    maxDisparity = 1
    fps = 0.0
    fps_frames = 0
    fps_t0 = time.perf_counter()
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        confidence = confidenceQueue.get()
        assert isinstance(disparity, dai.ImgFrame)
        assert isinstance(confidence, dai.ImgFrame)
        npDisparity = disparity.getFrame()
        npConfidence = confidence.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        fps_frames += 1
        now = time.perf_counter()
        dt = now - fps_t0
        if dt >= 1.0:
            fps = fps_frames / dt
            fps_frames = 0
            fps_t0 = now
        cv2.putText(
            colorizedDisparity,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("disparity", colorizedDisparity)
        cv2.imshow("confidence", npConfidence)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
