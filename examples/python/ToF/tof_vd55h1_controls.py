#!/usr/bin/env python3
"""Tune every RVC4 VD55H1 ToF IPP control with OpenCV sliders.

Trackbars ending in "x100" use a scale factor of 100. Toggle trackbars use
0 for off and 1 for on. Press q to quit.
"""

import cv2
import depthai as dai


FPS = 30.0
WINDOW = "VD55H1 controls"


def config_from_trackbars() -> dai.ToFConfig:
    config = dai.ToFConfig()
    vd55h1 = config.vd55h1
    vd55h1.phaseUnwrapErrorThreshold = cv2.getTrackbarPos("unwrap threshold", WINDOW)
    vd55h1.enableBilateralFilter = bool(cv2.getTrackbarPos("bilateral", WINDOW))
    vd55h1.bilateralStdFactor = cv2.getTrackbarPos("bilateral std x100", WINDOW) / 100.0
    vd55h1.bilateralKernelSize = cv2.getTrackbarPos("bilateral kernel", WINDOW)
    vd55h1.enableTemporalNoiseReduction = bool(cv2.getTrackbarPos("temporal NR", WINDOW))
    vd55h1.temporalNoiseReductionMaxGain = cv2.getTrackbarPos("TNR max gain", WINDOW)
    vd55h1.temporalNoiseReductionStdFactor = cv2.getTrackbarPos("TNR std x100", WINDOW) / 100.0
    vd55h1.enableFlyingPixelFilter = bool(cv2.getTrackbarPos("flying pixel", WINDOW))
    vd55h1.flyingPixelDepthThreshold = cv2.getTrackbarPos("FP depth threshold", WINDOW)
    vd55h1.flyingPixelMinDepthOccurrence = cv2.getTrackbarPos("FP min occurrence x100", WINDOW) / 100.0
    vd55h1.enableRadialToPerpendicularCorrection = bool(cv2.getTrackbarPos("IPP radial-to-perp", WINDOW))
    return config


def main() -> None:
    with dai.Pipeline() as pipeline:
        tof = pipeline.create(dai.node.ToF).build(
            boardSocket=dai.CameraBoardSocket.AUTO,
            profile=dai.ToFConfig.Profile.MID_RANGE,
            fps=FPS,
        )
        depth_queue = tof.depth.createOutputQueue(maxSize=1, blocking=False)
        config_queue = tof.tofBaseInputConfig.createInputQueue()

        cv2.namedWindow(WINDOW)
        controls = (
            ("unwrap threshold", 500, 192),
            ("bilateral", 1, 1),
            ("bilateral std x100", 1000, 205),
            ("bilateral kernel", 15, 5),
            ("temporal NR", 1, 1),
            ("TNR max gain", 100, 27),
            ("TNR std x100", 500, 82),
            ("flying pixel", 1, 1),
            ("FP depth threshold", 1000, 101),
            ("FP min occurrence x100", 5000, 1356),
            ("IPP radial-to-perp", 1, 0),
        )
        for name, maximum, initial in controls:
            cv2.createTrackbar(name, WINDOW, initial, maximum, lambda _: None)

        pipeline.start()
        previous = None
        while pipeline.isRunning():
            values = tuple(cv2.getTrackbarPos(name, WINDOW) for name, _, _ in controls)
            if values != previous:
                config_queue.send(config_from_trackbars())
                previous = values

            frame = depth_queue.tryGet()
            if frame is not None:
                cv2.imshow("ToF depth", dai.utility.colorizeDepthFrame(frame, useLog=True).getCvFrame())

            if cv2.waitKey(1) == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
