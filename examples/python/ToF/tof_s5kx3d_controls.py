#!/usr/bin/env python3
"""Tune RVC2 S5K33D/S5K63D ToF controls with OpenCV sliders.

Unwrap level 0 disables phase unwrapping. The level range 0-5 and threshold range 0-500 mm are
example tuning ranges.
S5K63D/s5k63d are aliases of S5K33D/s5k33d, sharing the same settings.
Press q to quit.
"""

import cv2
import depthai as dai


FPS = 30.0
WINDOW = "S5K33D / S5K63D controls"


def configFromTrackbars(sensorName: str) -> dai.ToFConfig:
    config = dai.ToFConfig()
    # Both names access the same settings; choose the detected sensor's name.
    params = config.s5k63d if sensorName == "S5K63D" else config.s5k33d
    params.phaseUnwrappingLevel = cv2.getTrackbarPos("unwrap level", WINDOW)
    params.phaseUnwrapErrorThreshold = cv2.getTrackbarPos("unwrap threshold mm", WINDOW)
    return config


def main() -> None:
    with dai.Pipeline() as pipeline:
        cameras = pipeline.getDefaultDevice().getConnectedCameraFeatures()
        sensor = next((camera for camera in cameras if camera.sensorName in ("S5K33D", "S5K63D")), None)
        if sensor is None:
            sensorNames = ", ".join(camera.sensorName for camera in cameras) or "none"
            raise RuntimeError(f"This example requires an S5K33D or S5K63D ToF sensor. Found sensors: {sensorNames}")
        tof = pipeline.create(dai.node.ToF).build(
            boardSocket=sensor.socket,
            profile=dai.ToFConfig.Profile.MID_RANGE,
            fps=FPS,
        )
        depthQueue = tof.depth.createOutputQueue(maxSize=1, blocking=False)
        configQueue = tof.tofBaseInputConfig.createInputQueue()

        cv2.namedWindow(WINDOW)
        controls = (
            ("unwrap level", 5, 4),
            ("unwrap threshold mm", 500, 75),  # MID_RANGE preset threshold.
        )
        for name, maximum, initial in controls:
            cv2.createTrackbar(name, WINDOW, initial, maximum, lambda _: None)

        tof.setInitialConfig(configFromTrackbars(sensor.sensorName))
        pipeline.start()
        previous = tuple(cv2.getTrackbarPos(name, WINDOW) for name, _, _ in controls)
        while pipeline.isRunning():
            values = tuple(cv2.getTrackbarPos(name, WINDOW) for name, _, _ in controls)
            if values != previous:
                configQueue.send(configFromTrackbars(sensor.sensorName))
                previous = values

            frame = depthQueue.tryGet()
            if frame is not None:
                cv2.imshow("ToF depth", dai.utility.colorizeDepthFrame(frame, useLog=True).getCvFrame())

            if cv2.waitKey(1) == ord("q"):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
