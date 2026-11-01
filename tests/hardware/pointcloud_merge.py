#!/usr/bin/env python3
"""Check live stereo merging against individual clouds of the exact same depth frames.

Run with the candidate bindings on PYTHONPATH. For example:
  python tests/hardware/pointcloud_merge.py --device 895405418 --pair B:C --pair C:E
  python tests/hardware/pointcloud_merge.py --device 19443010613B941700 --pair B:C

Only reads device calibration; it does not flash or replace it. All clouds use
CAM_B coordinates and organized output so every source pixel can be compared.
"""

import argparse
import json
from datetime import timedelta

import depthai as dai
import numpy as np


def socket(name):
    return getattr(dai.CameraBoardSocket, "CAM_" + name.removeprefix("CAM_"))


def getMessage(queue):
    message = queue.get(timedelta(seconds=20))
    if message is None:
        raise RuntimeError("Timed out waiting for a point-cloud test message")
    return message


def matchingCloud(queue, timestamp):
    for _ in range(100):
        cloud = getMessage(queue)
        if cloud.getTimestamp() == timestamp:
            return cloud
        if cloud.getTimestamp() > timestamp:
            raise AssertionError("Individual cloud skipped the depth frame used by the merge")
    raise AssertionError("Could not match individual cloud to merged depth frame")


def run(args):
    pairs = [tuple(socket(part) for part in pair.split(":")) for pair in args.pair]
    if any(len(pair) != 2 for pair in pairs):
        raise ValueError("Each --pair must be LEFT:RIGHT, for example B:C")
    with dai.Device(dai.DeviceInfo(args.device)) as device, dai.Pipeline(device) as pipeline:
        pipeline.setAutoCalibrationMode(dai.Pipeline.AutoCalibrationMode.OFF)
        calibration = device.readCalibration()
        features = device.getConnectedCameraFeatures()
        for left, right in pairs:
            transform = np.asarray(calibration.getCameraExtrinsics(left, right))
            if np.linalg.norm(transform[:3, 3]) < 0.1:
                raise RuntimeError(f"{left}/{right} has no usable calibrated baseline")
        outputs = {}
        for camSocket in sorted({s for pair in pairs for s in pair}, key=int):
            camera = pipeline.create(dai.node.Camera).build(camSocket, sensorFps=args.fps)
            outputs[camSocket] = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=args.fps)

        merged = pipeline.create(dai.node.PointCloud)
        merged.setRunOnHost(not args.onDevice)
        merged.sync.setRunOnHost(not args.onDevice)
        merged.initialConfig.setOrganized(True)
        merged.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
        merged.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_B)
        merged.sync.setSyncThreshold(timedelta(milliseconds=30))
        merged.setBackend(dai.Backend.GPU if args.gpu else dai.Backend.CPU).setNumThreads(args.threads)
        cloudQueue = merged.outputPointCloud.createOutputQueue(30, True)
        depthQueue = merged.passthroughDepth.createOutputQueue(60, True)
        individualQueues = []
        for index, (left, right) in enumerate(pairs):
            stereo = pipeline.create(dai.node.StereoDepth)
            stereo.setLeftRightCheck(True)
            stereo.setExtendedDisparity(False)
            outputs[left].link(stereo.left)
            outputs[right].link(stereo.right)
            stereo.depth.link(merged.getDepthInput(str(index)))
            individual = pipeline.create(dai.node.PointCloud)
            individual.setRunOnHost(True)
            individual.sync.setRunOnHost(True)
            individual.initialConfig.setOrganized(True)
            individual.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
            individual.setTargetCoordinateSystem(dai.CameraBoardSocket.CAM_B)
            stereo.depth.link(individual.inputDepth)
            individualQueues.append(individual.outputPointCloud.createOutputQueue(30, True))
        pipeline.start()
        validCounts = [0] * len(pairs)
        maximumError = 0.0
        firstTimestamp = None
        for _ in range(args.frames):
            cloud = getMessage(cloudQueue)
            depthFrames = [getMessage(depthQueue) for _ in pairs]
            expected = []
            for index, frame in enumerate(depthFrames):
                validCounts[index] += int(np.count_nonzero(frame.getFrame()))
                expected.append(matchingCloud(individualQueues[index], frame.getTimestamp()).getPoints())
            expectedPoints = np.concatenate(expected)
            points = cloud.getPoints()
            assert points.shape == expectedPoints.shape, (points.shape, expectedPoints.shape)
            assert cloud.getHeight() == 400 * len(pairs), cloud.getHeight()
            assert cloud.getWidth() == 640, cloud.getWidth()
            assert np.isfinite(points).all(), "Cloud contains non-finite coordinates"
            error = float(np.max(np.abs(points - expectedPoints)))
            maximumError = max(maximumError, error)
            np.testing.assert_allclose(points, expectedPoints, rtol=1e-5, atol=0.05)
            firstTimestamp = firstTimestamp or cloud.getTimestamp()
        assert all(count > 0 for count in validCounts), f"Stereo views produced empty depth: {validCounts}"
        elapsed = (cloud.getTimestamp() - firstTimestamp).total_seconds()
        result = {
            "device": device.getDeviceId(),
            "model": calibration.getEepromData().productName,
            "pairs": args.pair,
            "frames": args.frames,
            "validDepthPixels": validCounts,
            "maxCoordinateErrorMm": maximumError,
            "deliveredFps": (args.frames - 1) / elapsed if elapsed else 0,
            "onDevice": args.onDevice,
            "gpu": args.gpu,
            "sensors": [str(feature) for feature in features],
            "sdk": dai.__version__,
            "bindings": dai.__file__,
        }
        print(json.dumps(result), flush=True)
        pipeline.stop()
        return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", required=True)
    parser.add_argument("--pair", action="append", required=True)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--fps", type=float, default=5)
    parser.add_argument("--on-device", dest="onDevice", action="store_true")
    parser.add_argument("--gpu", action="store_true")
    parser.add_argument("--threads", type=int, default=1)
    run(parser.parse_args())
