#!/usr/bin/env python3

import cv2
import depthai as dai

# HDR Camera example
# Use 'h' and 'j' keys to increase/decrease HDR exposure ratio
# Use 'w' and 'e' keys to increase/decrease local tone weight
# Use 'b' to toggle HDR exposure base between 'long' and 'middle'
# Use 'q' to quit

# This example only works on IMX586/582 sensors.

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cameraFeatures = pipeline.getDefaultDevice().getConnectedCameraFeatures()
    success = False
    for feature in cameraFeatures:
        if feature.socket == dai.CameraBoardSocket.CAM_A:
            if feature.sensorName in ["IMX586", "IMX582", "LCM48"]:
                success = True
                break

    if not success:
        raise RuntimeError("No suitable camera found, HDR is only supported on IMX586/582 sensors!")

    cam.initialControl.setHdr(True)
    cameraControlQueue = cam.inputControl.createInputQueue()
    videoQueue = cam.requestOutput((1280,800)).createOutputQueue()

    minRatio = 1
    maxRatio = 8
    ratio = 4

    minWeight = 0.0
    maxWeight = 1.0
    weight = 0.75

    base = "long"

    pipeline.start()

    while pipeline.isRunning():
        def clamp(n, smallest, largest):
            return max(smallest, min(n, largest))
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        cv2.imshow("video", videoIn.getCvFrame())
        key = cv2.waitKey(1)

        ctrl = dai.CameraControl()
        if key == ord("h"):
            ratio = clamp(ratio * 2, minRatio, maxRatio)
            ctrl.setMisc("hdr-exposure-ratio", ratio)
            print(f"Setting HDR exposure ratio to {ratio}")
            cameraControlQueue.send(ctrl)
        if key == ord("j"):
            ratio = clamp(ratio // 2, minRatio, maxRatio)
            ctrl.setMisc("hdr-exposure-ratio", ratio)
            print(f"Setting HDR exposure ratio to {ratio}{' (off)' if ratio == 1 else ''}")
            cameraControlQueue.send(ctrl)
        if key == ord("w"):
            weight = clamp(weight + 1/32, minWeight, maxWeight)
            ctrl.setMisc("hdr-local-tone-weight", weight)
            print(f"Setting HDR local tone weight to {weight}")
            cameraControlQueue.send(ctrl)
        if key == ord("e"):
            weight = clamp(weight - 1/32, minWeight, maxWeight)
            ctrl.setMisc("hdr-local-tone-weight", weight)
            print(f"Setting HDR local tone weight to {weight}")
            cameraControlQueue.send(ctrl)
        if key == ord("b"):
            base = ("middle" if base == "long" else "long")
            ctrl.setMisc("hdr-exposure-base", base)
            print(f"Setting HDR exposure base to: {base}")
            cameraControlQueue.send(ctrl)
        if key == ord("q"):
            break
