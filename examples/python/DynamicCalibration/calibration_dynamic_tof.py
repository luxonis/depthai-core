#!/usr/bin/env python3
"""Refine native ToF depth/amplitude extrinsics against a camera on the host.

Keys: a=apply candidate in memory, r=collect again, q=quit. No EEPROM writes.
"""

import argparse
from datetime import timedelta

import numpy as np

import cv2
import depthai as dai


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="Device IP or ID; defaults to the first available device")
    parser.add_argument("--camera", choices=["CAM_B", "CAM_C"], default="CAM_C", help="Target camera (default: CAM_C)")
    parser.add_argument("--exposure-us", type=int, default=0, help="Manual camera exposure at ISO 100; 0 keeps auto exposure")
    args = parser.parse_args()

    device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
    with device, dai.Pipeline(device) as pipeline:
        features = device.getConnectedCameraFeatures()
        cameraSocket = getattr(dai.CameraBoardSocket, args.camera)
        if not any(dai.CameraSensorType.TOF in feature.supportedTypes for feature in features):
            raise RuntimeError("This device has no ToF sensor; use the image-based calibration example")
        if not any(feature.socket == cameraSocket for feature in features):
            raise RuntimeError(f"This device has no {args.camera} camera")
        pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
        fps = 15
        tof = pipeline.create(dai.node.ToF).build(dai.CameraBoardSocket.AUTO, dai.ToFConfig.Profile.MID_RANGE, fps)
        camera = pipeline.create(dai.node.Camera).build(cameraSocket)
        if args.exposure_us > 0:
            camera.initialControl.setManualExposure(args.exposure_us, 100)
        cameraOut = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=fps, enableUndistortion=False)

        dyn = pipeline.create(dai.node.DynamicCalibration)
        dyn.setRunOnHost(True)
        dyn.sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
        # The metric depth packet belongs to the ToF sensor; it is not another camera.
        tof.amplitude.link(dyn.inputs["tof"])
        cameraOut.link(dyn.left)
        tof.depth.link(dyn.tofDepth)

        previews = {
            "ToF amplitude": tof.amplitude.createOutputQueue(maxSize=1, blocking=False),
            args.camera: cameraOut.createOutputQueue(maxSize=1, blocking=False),
        }
        # ImageAlign produces undistorted depth. Use a separate undistorted camera
        # output for the preview; calibration above consumes the raw camera model.
        previewCamera = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=fps, enableUndistortion=True)
        align = pipeline.create(dai.node.ImageAlign)
        align.setRunOnHost(True)
        tof.depth.link(align.input)
        previewCamera.link(align.inputAlignTo)
        previewSync = pipeline.create(dai.node.Sync)
        previewSync.setRunOnHost(True)
        previewSync.setSyncThreshold(timedelta(seconds=0.5 / fps))
        previewCamera.link(previewSync.inputs["camera"])
        align.outputAligned.link(previewSync.inputs["depth"])
        overlayQueue = previewSync.out.createOutputQueue(maxSize=1, blocking=False)
        coverageQueue = dyn.coverageOutput.createOutputQueue(maxSize=4, blocking=False)
        calibrationQueue = dyn.calibrationOutput.createOutputQueue(maxSize=4, blocking=False)
        control = dyn.inputControl.createInputQueue()

        pipeline.start()
        control.send(dai.DynamicCalibrationControl.setPerformanceMode(
            dai.DynamicCalibrationControl.PerformanceMode.OPTIMIZE_SPEED
        ))
        control.send(dai.DynamicCalibrationControl.startCalibration())
        candidate = None
        print("Native metric ToF edge refinement on host", flush=True)
        print("Collect common edges at several depths and orientations. Keys: a=apply candidate in memory, r=collect again, q=quit.", flush=True)

        while pipeline.isRunning():
            for name, queue in previews.items():
                frame = queue.tryGet()
                if frame is not None:
                    image = frame.getCvFrame()
                    if name == "ToF amplitude":
                        # Display scaling; the node handles native amplitude and metric depth.
                        high = max(float(np.percentile(image, 99)), 1.0)
                        image = np.uint8(np.clip(image.astype(float) * 255.0 / high, 0, 255))
                    cv2.imshow(name, image)

            overlay = overlayQueue.tryGet()
            if overlay is not None:
                gray = cv2.cvtColor(overlay["camera"].getCvFrame(), cv2.COLOR_GRAY2BGR)
                depth = overlay["depth"].getCvFrame()
                colors = cv2.applyColorMap(np.uint8(np.clip(depth.astype(float) * 255.0 / 4000.0, 0, 255)), cv2.COLORMAP_TURBO)
                blended = cv2.addWeighted(gray, 0.5, colors, 0.5, 0)
                blended[depth == 0] = gray[depth == 0]
                cv2.imshow(f"Depth -> {args.camera} (runtime calibration)", blended)

            coverage = coverageQueue.tryGet()
            if coverage is not None:
                print(
                    f"Collection progress: {coverage.dataAcquired:.1f}%",
                    flush=True,
                )

            result = calibrationQueue.tryGet()
            if result is not None:
                print(f"Calibration: {result.info}", flush=True)
                if result.calibrationData is not None:
                    candidate = result.calibrationData.newCalibration
                    quality = result.calibrationData.calibrationDifference
                    print(f"Sampson error: {quality.sampsonErrorCurrent:.3f} -> {quality.sampsonErrorNew:.3f} px", flush=True)
                    for pair, rotation in quality.pairwiseRotationDifference.items():
                        print(f"Rotation change {pair}: {list(rotation)} deg", flush=True)
                    print("Candidate ready. Press a to apply it in memory or r to collect a different view.", flush=True)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("a") and candidate is not None:
                control.send(dai.DynamicCalibrationControl.applyCalibration(candidate, flash=False))
                candidate = None
                print("Requested runtime calibration update; EEPROM unchanged.", flush=True)
            if key == ord("r"):
                candidate = None
                control.send(dai.DynamicCalibrationControl.resetData())
                control.send(dai.DynamicCalibrationControl.startCalibration())

        pipeline.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
