#!/usr/bin/env python3

import cv2
import depthai as dai

# HDR Camera example
# Use 'h' and 'j' keys to increase/decrease HDR exposure ratio
# Use 'w' and 'e' keys to increase/decrease local tone weight

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
    expTime = 20000
    sensIso = 400

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
        elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
            if key == ord('i'): expTime -= 1000
            if key == ord('o'): expTime += 1000
            if key == ord('k'): sensIso -= 100
            if key == ord('l'): sensIso += 100
            expTime = clamp(expTime, 1000, 100000)
            sensIso = clamp(sensIso, 100, 1600)
            print("Setting manual exposure, time: ", expTime, "iso: ", sensIso)
            ctrl.setManualExposure(expTime, sensIso)
            cameraControlQueue.send(ctrl)
        if key == ord("q"):
            break
