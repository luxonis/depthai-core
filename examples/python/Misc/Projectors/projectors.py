#!/usr/bin/env python3

import cv2
import depthai as dai

dot_intensity = 1
DOT_STEP = 0.1

flood_intensity = 1
FLOOD_STEP = 0.1

# Create pipeline
device = dai.Device()
with dai.Pipeline(device) as pipeline:
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # Linking
    monoLeftOut = monoLeft.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
    monoRightOut = monoRight.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)

    leftQueue = monoLeftOut.createOutputQueue()
    rightQueue = monoRightOut.createOutputQueue()

    pipeline.start()
    pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(dot_intensity)
    pipeline.getDefaultDevice().setIrFloodLightIntensity(flood_intensity)
    while pipeline.isRunning():
        leftSynced = leftQueue.get()
        rightSynced = rightQueue.get()
        assert isinstance(leftSynced, dai.ImgFrame)
        assert isinstance(rightSynced, dai.ImgFrame)
        cv2.imshow(f"left", leftSynced.getCvFrame())
        cv2.imshow(f"right", rightSynced.getCvFrame())

        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
        elif key == ord("w"):
            dot_intensity += DOT_STEP
            if dot_intensity > 1:
                dot_intensity = 1
            pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(dot_intensity)
            print(f"Dot intensity: {dot_intensity}")
        elif key == ord("s"):
            dot_intensity -= DOT_STEP
            if dot_intensity < 0:
                dot_intensity = 0
            pipeline.getDefaultDevice().setIrLaserDotProjectorIntensity(dot_intensity)
            print(f"Dot intensity: {dot_intensity}")
        elif key == ord("a"):
            flood_intensity += FLOOD_STEP
            if flood_intensity > 1:
                flood_intensity = 1
            pipeline.getDefaultDevice().setIrFloodLightIntensity(flood_intensity)
            print(f"Flood intensity: {flood_intensity}")
        elif key == ord("d"):
            flood_intensity -= FLOOD_STEP
            if flood_intensity < 0:
                flood_intensity = 0
            pipeline.getDefaultDevice().setIrFloodLightIntensity(flood_intensity)
            print(f"Flood intensity: {flood_intensity}")
