#!/usr/bin/env python3
"""Exercise ToF calibration, injected runtime error, candidate apply and restore.

Run explicitly against one device in a static scene with shared edges at several
depths. A rejection fails the test; it can indicate insufficient scene evidence.
The test never flashes EEPROM and restores the original runtime calibration.
"""

import argparse
from datetime import timedelta
import json
from pathlib import Path
import time

import cv2
import depthai as dai
import numpy as np


def transform(calibration, source, target):
    value = np.asarray(calibration.getCameraExtrinsics(source, target), dtype=float)
    value[:3, 3] *= 0.01
    return value


def perturb(calibration, source, target, pitch_degrees):
    result = dai.CalibrationHandler(calibration.getEepromData())
    desired = transform(calibration, source, target)
    desired[:3, :3] = cv2.Rodrigues(np.array([np.deg2rad(pitch_degrees), 0.0, 0.0]))[0] @ desired[:3, :3]
    # Update every incident EEPROM link so moving the ToF never moves the rest
    # of the rig, irrespective of which direction the stored chain traverses it.
    for socket, camera in calibration.getEepromData().cameraData.items():
        destination = camera.extrinsics.toCameraSocket
        if destination == dai.CameraBoardSocket.AUTO:
            continue
        if socket == source:
            edge = transform(calibration, target, destination) @ desired
        elif destination == source:
            edge = np.linalg.inv(desired) @ transform(calibration, socket, target)
        else:
            continue
        result.updateCameraExtrinsics(socket, destination, edge[:3, :3].tolist(), (100 * edge[:3, 3]).tolist())
    return result


def apply_and_wait(control, device, calibration, source, target):
    control.send(dai.DynamicCalibrationControl.applyCalibration(calibration, flash=False))
    expected = transform(calibration, source, target)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if np.allclose(transform(device.getCalibration(), source, target), expected, atol=1e-5):
            return
        time.sleep(0.05)
    raise AssertionError("Runtime calibration did not propagate")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="Device IP or ID")
    parser.add_argument("--camera", choices=["CAM_B", "CAM_C"], default="CAM_C")
    parser.add_argument("--pitch-deg", type=float, default=-0.5)
    parser.add_argument("--exposure-us", type=int, help="Optional manual target exposure at ISO 100")
    parser.add_argument("--timeout", type=float, default=60)
    parser.add_argument("--output", type=Path, required=True, help="JSON test report")
    args = parser.parse_args()
    report = {"device": args.device, "target": args.camera, "injected_pitch_deg": args.pitch_deg, "accepted": False}
    try:
        with dai.Device(dai.DeviceInfo(args.device)) as device, dai.Pipeline(device) as pipeline:
            original = device.getCalibration()
            eeprom_before = device.readCalibration().eepromToJson()
            features = device.getConnectedCameraFeatures()
            tof_sockets = [feature.socket for feature in features if dai.CameraSensorType.TOF in feature.supportedTypes]
            assert len(tof_sockets) == 1, "Expected exactly one ToF sensor"
            source, target = tof_sockets[0], getattr(dai.CameraBoardSocket, args.camera)
            assert any(feature.socket == target for feature in features), "Target camera is unavailable"
            report.update(product=device.getDeviceName(), sdk=dai.__version__, tof_socket=str(source))
            pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
            fps = 15
            tof = pipeline.create(dai.node.ToF).build(source, dai.ToFConfig.Profile.MID_RANGE, fps)
            camera = pipeline.create(dai.node.Camera).build(target)
            if args.exposure_us:
                camera.initialControl.setManualExposure(args.exposure_us, 100)
            image = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=fps, enableUndistortion=False)
            node = pipeline.create(dai.node.DynamicCalibration)
            node.setRunOnHost(True)
            node.sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
            tof.amplitude.link(node.inputs["tof"])
            tof.depth.link(node.tofDepth)
            image.link(node.left)
            results = node.calibrationOutput.createOutputQueue(maxSize=4, blocking=False)
            control = node.inputControl.createInputQueue()
            pipeline.start()
            try:
                damaged = perturb(original, source, target, args.pitch_deg)
                apply_and_wait(control, device, damaged, source, target)
                control.send(dai.DynamicCalibrationControl.startCalibration())
                deadline = time.monotonic() + args.timeout
                messages = []
                candidate = None
                while pipeline.isRunning() and time.monotonic() < deadline:
                    result = results.tryGet()
                    if result is None:
                        time.sleep(0.02)
                        continue
                    messages.append(result.info)
                    print(result.info, flush=True)
                    if result.calibrationData is not None:
                        candidate = result.calibrationData.newCalibration
                        quality = result.calibrationData.calibrationDifference
                        report.update(sampson_before_px=quality.sampsonErrorCurrent, sampson_after_px=quality.sampsonErrorNew)
                        break
                report["messages"] = messages
                assert candidate is not None, "No accepted calibration; inspect the scene and rejection messages"
                for socket, data in original.getEepromData().cameraData.items():
                    assert np.array_equal(candidate.getCameraIntrinsics(socket), original.getCameraIntrinsics(socket))
                    assert candidate.getDistortionCoefficients(socket) == original.getDistortionCoefficients(socket)
                fixed = [socket for socket in original.getEepromData().cameraData if socket != source]
                for first in fixed:
                    for second in fixed:
                        assert np.allclose(transform(candidate, first, second), transform(original, first, second), atol=1e-5)
                apply_and_wait(control, device, candidate, source, target)
                report.update(accepted=True, runtime_applied=True, fixed_rig_preserved=True)
            finally:
                apply_and_wait(control, device, original, source, target)
                report["runtime_restored"] = True
                report["eeprom_unchanged"] = device.readCalibration().eepromToJson() == eeprom_before
                assert report["eeprom_unchanged"], "EEPROM changed"
                pipeline.stop()
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
