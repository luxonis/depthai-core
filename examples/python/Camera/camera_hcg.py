#!/usr/bin/env python3

import cv2
import depthai as dai

# High conversion gain (HCG) camera example
# Use 'h' to toggle high conversion gain on/off
# Use 'q' to quit

# HCG raises the sensor's conversion gain (about 2.6x on IMX462): less noise in low light,
# but pixels saturate about 2.6x sooner. Auto exposure compensates for the extra signal,
# so watch the exposure and ISO shown on the frame drop when HCG is enabled.

# This example only works on IMX462 sensors.

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cameraFeatures = pipeline.getDefaultDevice().getConnectedCameraFeatures()
    success = False
    for feature in cameraFeatures:
        if feature.socket == dai.CameraBoardSocket.CAM_A and feature.sensorName == "IMX462":
            success = True
            break

    if not success:
        raise RuntimeError("No suitable camera found, high conversion gain is only supported on IMX462 sensors!")

    hcg = False
    cam.initialControl.setMisc("high-conversion-gain", int(hcg))
    cameraControlQueue = cam.inputControl.createInputQueue()
    videoQueue = cam.requestOutput((1280, 720)).createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        videoIn = videoQueue.get()
        assert isinstance(videoIn, dai.ImgFrame)
        frame = videoIn.getCvFrame()
        exposureMs = videoIn.getExposureTime().total_seconds() * 1000
        text = f"HCG {'on' if hcg else 'off'}, exposure {exposureMs:.2f} ms, ISO {videoIn.getSensitivity()}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("video", frame)
        key = cv2.waitKey(1)

        if key == ord("h"):
            hcg = not hcg
            ctrl = dai.CameraControl()
            ctrl.setMisc("high-conversion-gain", int(hcg))
            cameraControlQueue.send(ctrl)
            print(f"High conversion gain {'enabled' if hcg else 'disabled'}")
        if key == ord("q"):
            break
