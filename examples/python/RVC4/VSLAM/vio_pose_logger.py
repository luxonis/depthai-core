#!/usr/bin/env python3
"""Launch RVC4 firmware VIO from a PC, display poses, and log them to CSV."""

import argparse
from collections import deque
import csv
from datetime import timedelta
import math
import sys
import time

import depthai as dai


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", nargs="?", default="poses.csv", help="CSV file on this PC (overwritten)")
    parser.add_argument("--visualize", action="store_true", help="Show a 3D trajectory using rerun-sdk")
    args = parser.parse_args()
    if not hasattr(dai.node, "VIO"):
        raise RuntimeError("Install the updated core Python bindings containing dai.node.VIO.")
    if args.visualize:
        import rerun as rr
        rr.init("RVC4 VIO", spawn=True)
        rr.log("world", rr.ViewCoordinates.FLU)
    positions = deque(maxlen=5000)

    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        if device.getPlatform() != dai.Platform.RVC4:
            raise RuntimeError("This example requires RVC4 with the updated VIO firmware.")
        print(f"OAK OS: {device.getOSVersion()}")
        calibration = pipeline.getCalibrationData()
        try:
            for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
                calibration.getCameraToImuExtrinsics(socket, useSpecTranslation=True, unit=dai.LengthUnit.METER)
        except RuntimeError as error:
            raise RuntimeError(
                "VIO calibration check failed before starting cameras. Rebuild/select the RVC4 firmware "
                "with the IMU defaults fix, or supply valid camera-to-IMU calibration for this board. "
                f"Details: {error}"
            ) from error
        requestedFps = 90
        fps = requestedFps
        cameraFeatures = {camera.socket: camera for camera in device.getConnectedCameraFeatures()}
        for socket in (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C):
            camera = cameraFeatures.get(socket)
            if camera is None:
                raise RuntimeError(f"Stereo camera {socket} is not connected.")
            maxFps = max((config.maxFps for config in camera.configs
                          if config.width >= 640 and config.height >= 400 and not getattr(config, "hdr", False)), default=0)
            if maxFps <= 0:
                raise RuntimeError(f"{socket} ({camera.sensorName}) has no supported 640x400 camera mode.")
            fps = min(fps, maxFps)
        print(f"Stereo input: 640x400 at {fps:g} FPS (requested {requestedFps} FPS).")
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        imu = pipeline.create(dai.node.IMU)
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(False)
        sync.setTimestampSource(dai.node.Sync.TimestampSource.DEVICE)
        sync.setSyncThreshold(timedelta(milliseconds=5))
        vio = pipeline.create(dai.node.VIO)
        vio.setImuUpdateRate(200)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        left.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=fps, enableUndistortion=False).link(sync.inputs["left"])
        right.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=fps, enableUndistortion=False).link(sync.inputs["right"])
        sync.out.link(vio.stereo)
        imu.out.link(vio.imu)
        # This is the only data stream to the PC. No image or IMU output queues.
        poses = vio.transform.createOutputQueue(maxSize=8, blocking=False)

        with open(args.output, "w", newline="", buffering=1) as output:
            writer = csv.writer(output)
            writer.writerow(["sequence", "sample_time_s", "device_time_s", "system_time_s", "age_s",
                             "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"])
            pipeline.start()
            print(f"VIO runs on the RVC4 CPU; logging poses to {args.output}. Ctrl+C to stop.")
            print("Waiting for stereo and IMU startup; the first pose can take several seconds.")
            lastReceive = time.monotonic()
            lastDisplay = 0.0
            previousSample = 0.0
            while pipeline.isRunning():
                pose = poses.tryGet()
                now = time.monotonic()
                if pose is None:
                    timeout = 30.0 if previousSample == 0.0 else 2.0
                    if now - lastReceive > timeout:
                        raise RuntimeError("No VIO poses received; check firmware logs, calibration, and IMU.")
                    time.sleep(0.005)
                    continue
                sampleTime = pose.getTimestamp().total_seconds()
                deviceTime = pose.getTimestampDevice().total_seconds()
                age = dai.Clock.now().total_seconds() - sampleTime
                if deviceTime <= previousSample or not -0.05 <= age <= 2.0:
                    raise RuntimeError("Stale or invalid VIO timestamp; treat VIO as unavailable.")
                translation = pose.getTranslation()
                rotation = pose.getQuaternion()
                values = [translation.x, translation.y, translation.z, rotation.qx, rotation.qy, rotation.qz, rotation.qw]
                if not all(math.isfinite(value) for value in values):
                    raise RuntimeError("Non-finite VIO pose; treat VIO as unavailable.")
                systemTime = pose.getTimestampSystem()
                writer.writerow([pose.getSequenceNum(), f"{sampleTime:.9f}", f"{deviceTime:.9f}",
                                 "" if systemTime is None else f"{systemTime.timestamp():.9f}",
                                 f"{age:.6f}", *[f"{value:.9f}" for value in values]])
                if now - lastDisplay >= 0.2:
                    print(f"xyz [m]: {translation.x:+.3f} {translation.y:+.3f} {translation.z:+.3f}  "
                          f"q [xyzw]: {rotation.qx:+.3f} {rotation.qy:+.3f} {rotation.qz:+.3f} {rotation.qw:+.3f}  "
                          f"age: {age * 1000:.0f} ms")
                    if args.visualize:
                        positions.append(values[:3])
                        rr.log("world/camera", rr.Transform3D(translation=values[:3], rotation=rr.datatypes.Quaternion(xyzw=values[3:])))
                        rr.log("world/trajectory", rr.LineStrips3D(rr.components.LineStrip3D(list(positions))))
                    lastDisplay = now
                previousSample = deviceTime
                lastReceive = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.", file=sys.stderr)
    except (RuntimeError, OSError, ImportError) as error:
        sys.exit(str(error))
