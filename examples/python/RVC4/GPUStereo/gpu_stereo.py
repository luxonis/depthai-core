#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np

device = dai.Device()
if not device.isGpuStereoSupported():
    print("Exiting GPUStereo example: GPUStereo is not supported on this device.")
    raise SystemExit(0)

with dai.Pipeline(device) as pipeline:
    camLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    gpu = pipeline.create(dai.node.GPUStereo)
    camLeft.requestFullResolutionOutput().link(gpu.left)
    camRight.requestFullResolutionOutput().link(gpu.right)
    gpu.initialConfig.setConfidenceThreshold(25)

    dispQ = gpu.disparity.createOutputQueue()

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            frame = dispQ.get()
            d = frame.getFrame().astype(np.float32)
            d = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            cv2.imshow("GPUStereo Disparity", cv2.applyColorMap(d, cv2.COLORMAP_JET))
            if cv2.waitKey(1) == ord("q"):
                break
