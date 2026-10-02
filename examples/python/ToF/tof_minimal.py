#!/usr/bin/env python3
"""Minimal ToF script showing the main output stream.

Displays undistorted left camera (CAM_B) and depth images side by side.
Min/max depth sliders adjust the coloring range in millimeters.
For more streams, see tof_all_queues.py.
On RVC4, select a VD55H1 mode with --sensor-mode FREQUENCY_2_BINNED.
The RVC4 exposure slider sets manual exposure in microseconds (fixed ISO 100).
Set Auto exposure to 1 to enable AE, or 0 to use the manual exposure slider.
Moving the exposure slider switches back to manual mode.
The footer shows exposure times reported by the received left and ToF frames.

Press 'q' to quit.
"""

import argparse

import cv2
import depthai as dai
import numpy as np

FPS = 30.0


def depthLegend(height, minDepth, maxDepth):
    legend = np.zeros((height, 130, 3), dtype=np.uint8)
    gradient = np.linspace(255, 0, height - 40).round().astype(np.uint8)[:, None]
    legend[20:height - 20, 8:28] = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)
    for fraction in np.linspace(0, 1, 5):
        y = round(20 + fraction * (height - 41))
        value = maxDepth - fraction * (maxDepth - minDepth)
        cv2.putText(legend, f"{value:.0f} mm", (34, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return legend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sensor-mode",
        choices=list(dai.node.ToF.SensorMode.__members__),
        default="FREQUENCY_3",
        help="VD55H1 startup mode (RVC4 only; leave the default on RVC2)",
    )
    args = parser.parse_args()

    pipeline = dai.Pipeline()

    profile = dai.ToFConfig.Profile.MID_RANGE

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=profile,
        fps=FPS,
        sensorMode=dai.node.ToF.SensorMode.__members__[args.sensor_mode],
    )
    tof.setOutputUndistortion(True)

    depthOutputQueue = tof.depth.createOutputQueue(maxSize=1, blocking=False)
    # RVC4 ToF creates a Camera subnode for the raw sensor stream.
    camera = next(
        (node for node in pipeline.getAllNodes()
         if isinstance(node, dai.node.Camera) and node.getBoardSocket() == tof.tofBaseNode.getBoardSocket()),
        None,
    )
    controlQueue = camera.inputControl.createInputQueue() if camera is not None else None
    leftCamera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    leftFeatures = next(
        feature for feature in pipeline.getDefaultDevice().getConnectedCameraFeatures()
        if feature.socket == dai.CameraBoardSocket.CAM_B
    )
    leftQueue = leftCamera.requestOutput(
        (leftFeatures.width, leftFeatures.height),
        fps=FPS,
        resizeMode=dai.ImgResizeMode.LETTERBOX,
        enableUndistortion=True,
        alphaScaling=1.0,  # Preserve the full view, including black borders after undistortion.
    ).createOutputQueue(maxSize=1, blocking=False)

    with pipeline as p:
        p.start()
        cv2.namedWindow("depth", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow("depth", 1200, 600)
        cv2.createTrackbar("Min depth (mm)", "depth", 0, 14999, lambda _: None)
        cv2.createTrackbar("Max depth (mm)", "depth", 300, 15000, lambda _: None)
        cv2.setTrackbarMin("Max depth (mm)", "depth", 1)
        if controlQueue is not None:
            cv2.createTrackbar("Auto exposure", "depth", 0, 1, lambda _: None)
            cv2.createTrackbar("Exposure (us)", "depth", 100, 197, lambda _: None)
        lastExposure = None
        lastAutoExposure = None
        depthFrame = None
        leftFrame = None
        while p.isRunning():
            minDepth = cv2.getTrackbarPos("Min depth (mm)", "depth")
            maxDepth = cv2.getTrackbarPos("Max depth (mm)", "depth")
            if maxDepth <= minDepth:
                maxDepth = minDepth + 1
                cv2.setTrackbarPos("Max depth (mm)", "depth", maxDepth)

            if controlQueue is not None:
                autoExposure = bool(cv2.getTrackbarPos("Auto exposure", "depth"))
                exposure = cv2.getTrackbarPos("Exposure (us)", "depth")
                if lastExposure is not None and exposure != lastExposure and autoExposure:
                    autoExposure = False
                    cv2.setTrackbarPos("Auto exposure", "depth", 0)
                if autoExposure != lastAutoExposure or (not autoExposure and exposure != lastExposure):
                    control = dai.CameraControl()
                    if autoExposure:
                        control.setAutoExposureEnable()
                    else:
                        control.setManualExposure(exposure, 100)
                    controlQueue.send(control)
                    lastExposure = exposure
                    lastAutoExposure = autoExposure

            depth = depthOutputQueue.tryGet()
            if depth is not None:
                depthFrame = depth

            left = leftQueue.tryGet()
            if left is not None:
                leftExposureUs = round(left.getExposureTime().total_seconds() * 1_000_000)
                leftFrame = left.getCvFrame()
                if leftFrame.ndim == 2:
                    leftFrame = cv2.cvtColor(leftFrame, cv2.COLOR_GRAY2BGR)
                # Scale the full sensor image, preserving its aspect ratio and all edges.
                width = round(leftFrame.shape[1] * 400 / leftFrame.shape[0])
                leftFrame = cv2.resize(leftFrame, (width, 400), interpolation=cv2.INTER_AREA)

            if depthFrame is not None and leftFrame is not None:
                depthColor = dai.utility.colorizeDepthFrame(
                    depthFrame, minDepth, maxDepth, colormap=cv2.COLORMAP_JET, useLog=False
                ).getCvFrame()
                # Match display heights while preserving the complete depth image's aspect ratio.
                height = leftFrame.shape[0]
                width = round(depthColor.shape[1] * height / depthColor.shape[0])
                depthColor = cv2.resize(depthColor, (width, height), interpolation=cv2.INTER_NEAREST)
                tofExposureUs = round(depthFrame.getExposureTime().total_seconds() * 1_000_000)
                legend = depthLegend(height, minDepth, maxDepth)
                display = cv2.copyMakeBorder(cv2.hconcat([leftFrame, depthColor, legend]), 0, 32, 0, 0, cv2.BORDER_CONSTANT)
                for label, exposureUs, x in (("Left", leftExposureUs, 10), ("ToF", tofExposureUs, leftFrame.shape[1] + 10)):
                    exposureText = f"{exposureUs} us" if exposureUs > 0 else "unavailable"
                    cv2.putText(display, f"{label} exposure: {exposureText}", (x, height + 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow("depth", display)

            if cv2.waitKey(1) == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
