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
    # Supported input sizes: 1280x800 and 640x400 (rectified inside the node).
    gpu.build(camLeft.requestOutput((1280, 800), dai.ImgFrame.Type.GRAY8, fps=30),
              camRight.requestOutput((1280, 800), dai.ImgFrame.Type.GRAY8, fps=30))
    # Confidence 0..255, higher = more confident; pixels below the threshold are invalidated (0 = off).
    gpu.initialConfig.setConfidenceThreshold(10)
    # GPU_CPU (default) balances GPU and CPU; GPU_ONLY keeps the CPU almost free (GPU fully busy).
    gpu.initialConfig.setComputeMode(dai.GPUStereoConfig.ComputeMode.GPU_CPU)

    dispQ = gpu.disparity.createOutputQueue()

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            frame = dispQ.get()
            d = frame.getFrame().astype(np.float32)
            d = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            color = cv2.applyColorMap(d, cv2.COLORMAP_JET)
            color[frame.getFrame() == 0] = 0   # invalid pixels in black
            cv2.imshow("GPUStereo Disparity", color)
            if cv2.waitKey(1) == ord("q"):
                break
