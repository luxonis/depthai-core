#!/usr/bin/env python3
"""Apply DCL candidates immediately while viewing ToF aligned to rectified left.

Skips separate validation and defaults to no delay between calibration cycles.
DCL's own coverage/data checks still apply. Calibration changes remain in RAM.
Press q to exit; use the opacity slider to compare the image and depth edges.
Run with DEPTHAI_LEVEL=info for node progress.
"""

import argparse
from datetime import timedelta
import logging

import cv2 as cv
import depthai as dai


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="Device IP address or ID")
    parser.add_argument("--cycle-delay", type=int, default=0, help="Seconds between calibration cycles (default: 0)")
    parser.add_argument("--min-data-confidence", type=float, default=0.0, help="Apply DCL results with data confidence above this value (default: 0)")
    parser.add_argument("--max-images", type=int, default=100, help="Collection limit per attempt, not a minimum; apply as soon as a candidate is available")
    args = parser.parse_args()
    if args.cycle_delay < 0 or args.max_images < 1 or not 0.0 <= args.min_data_confidence <= 1.0:
        parser.error("Require --cycle-delay >= 0, --max-images >= 1, and --min-data-confidence between 0 and 1")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

    with dai.Device(dai.DeviceInfo(args.device)) as device, dai.Pipeline(device) as pipeline:
        device.setLogLevel(dai.LogLevel.INFO)
        device.setLogOutputLevel(dai.LogLevel.INFO)
        pairs = device.getStereoPairs()
        if not pairs:
            raise RuntimeError("The device has no stereo pair")
        pair = pairs[0]
        left = pipeline.create(dai.node.Camera).build(pair.left)
        right = pipeline.create(dai.node.Camera).build(pair.right)
        fps = 15.0
        rectification = pipeline.create(dai.node.Rectification)
        rectification.setRunOnHost(True)
        left.requestOutput((640, 400), fps=fps).link(rectification.input1)
        right.requestOutput((640, 400), fps=fps).link(rectification.input2)

        tof = pipeline.create(dai.node.ToF).build(
            boardSocket=dai.CameraBoardSocket.AUTO,
            profile=dai.ToFConfig.Profile.MID_RANGE,
            fps=fps,
        )
        align = pipeline.create(dai.node.Align)
        align.setRunOnHost(True)
        tof.depth.link(align.input)
        rectification.output1.link(align.inputAlignTo)

        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(True)
        sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
        rectification.output1.link(sync.inputs["left"])
        align.outputAligned.link(sync.inputs["tof"])
        frames = sync.out.createOutputQueue(maxSize=2, blocking=False)

        worker = pipeline.create(dai.node.AutoCalibration).build(left, right)
        worker.setLogLevel(dai.LogLevel.INFO)
        worker.initialConfig.mode = dai.AutoCalibrationConfig.CONTINUOUS
        worker.initialConfig.flashCalibration = False
        worker.initialConfig.sleepingTime = args.cycle_delay
        # This bypasses both the pre-check and the separate candidate validation.
        # DCL still has to return a successful result before anything is applied.
        worker.initialConfig.validationSetSize = 0
        worker.initialConfig.maxImagesPerRecalibration = args.max_images
        worker.initialConfig.dataConfidenceThreshold = args.min_data_confidence
        results = worker.output.createOutputQueue(maxSize=4, blocking=False)
        previousCalibration = device.getCalibration().eepromToJson()
        updates = 0
        status = "Collecting calibration data"

        window = f"ToF aligned to rectified left ({pair.left.name})"
        cv.namedWindow(window, cv.WINDOW_NORMAL)
        cv.createTrackbar("ToF opacity %", window, 50, 100, lambda _: None)
        pipeline.start()
        logging.info("Continuous autocalibration active; ToF aligned to %s; RAM-only updates", pair.left.name)
        logging.info("Fast acceptance: no separate validation; data confidence > %.3f; cycle delay %ds", args.min_data_confidence, args.cycle_delay)
        firstFrame = True
        while pipeline.isRunning():
            result = results.tryGet()
            if result is not None:
                if result.passed:
                    calibration = result.calibration.eepromToJson()
                    if calibration != previousCalibration:
                        updates += 1
                        previousCalibration = calibration
                        status = f"After calibration update {updates}"
                        logging.info("New calibration accepted and sent for application (update %d)", updates)
                    else:
                        status = f"Candidate accepted, unchanged | updates: {updates}"
                        logging.info("DCL candidate accepted; no parameter change")
                else:
                    status = f"Collecting / retrying | updates: {updates}"
                    logging.info("Calibration cycle did not pass; continuous mode will retry")

            group = frames.tryGet()
            if group is not None:
                image = group["left"].getCvFrame()
                if image.ndim == 2:
                    image = cv.cvtColor(image, cv.COLOR_GRAY2BGR)
                depth = group["tof"]
                color = dai.utility.colorizeDepthFrame(depth, 100, 7000, cv.COLORMAP_TURBO, useLog=True).getCvFrame()
                opacity = cv.getTrackbarPos("ToF opacity %", window) / 100.0
                overlay = cv.addWeighted(image, 1.0 - opacity, color, opacity, 0)
                invalid = depth.getFrame() == 0
                overlay[invalid] = image[invalid]
                cv.putText(overlay, status, (12, 28), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv.imshow(window, overlay)
                if firstFrame:
                    logging.info("Live synchronized ToF/left overlay is displaying")
                    firstFrame = False
            if cv.waitKey(10) == ord("q"):
                break
        pipeline.stop()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
