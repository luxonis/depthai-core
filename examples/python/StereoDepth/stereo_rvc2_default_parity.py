import argparse
import datetime
import os
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


SCENES = ("Adirondack-perfect", "Motorcycle-perfect", "Pipes-perfect")
SIZE = (1280, 800)


def parse_args():
    parser = argparse.ArgumentParser(description="Compare RVC4 DSP_RVC2_DEFAULT with physical RVC2 Middlebury captures")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("references", type=Path)
    parser.add_argument("--device", default=os.environ.get("DEPTHAI_DEVICE_NAME_LIST", "10.12.234.100"))
    parser.add_argument("--scenes", nargs="+", default=SCENES)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--timeout", type=float, default=180.0)
    return parser.parse_args()


def frame(image, sequence, socket):
    message = dai.ImgFrame()
    message.setData(image.reshape(-1))
    message.setTimestamp(datetime.timedelta(milliseconds=sequence * 50))
    message.setSequenceNum(sequence)
    message.setInstanceNum(socket)
    message.setType(dai.ImgFrame.Type.RAW8)
    message.setWidth(SIZE[0])
    message.setStride(SIZE[0])
    message.setHeight(SIZE[1])
    return message


def receive(queue, count, timeout):
    result = {}
    deadline = time.monotonic() + timeout
    while len(result) != count:
        message = queue.tryGet()
        if message is not None and message.getSequenceNum() < count:
            result[message.getSequenceNum()] = message.getFrame().copy()
        elif time.monotonic() >= deadline:
            raise TimeoutError(f"received {sorted(result)} of {count} frames")
        else:
            time.sleep(0.01)
    return result


def run_pair(pair, device, timeout):
    pipeline = dai.Pipeline(dai.Device(dai.DeviceInfo(device)))
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setStereoBackend(dai.StereoDepthProperties.StereoBackend.DSP_RVC2_DEFAULT)
    stereo.setInputResolution(*SIZE)
    stereo.setRectification(False)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    left_queue = stereo.left.createInputQueue()
    right_queue = stereo.right.createInputQueue()
    output_queue = stereo.disparity.createOutputQueue()
    rectified_left_queue = stereo.rectifiedLeft.createOutputQueue()
    rectified_right_queue = stereo.rectifiedRight.createOutputQueue()
    with pipeline:
        pipeline.start()
        left_queue.send(frame(pair[0], 0, dai.CameraBoardSocket.CAM_B))
        right_queue.send(frame(pair[1], 0, dai.CameraBoardSocket.CAM_C))
        output = receive(output_queue, 1, timeout)[0]
        rectified_left = receive(rectified_left_queue, 1, timeout)[0]
        rectified_right = receive(rectified_right_queue, 1, timeout)[0]
    return output, rectified_left, rectified_right


def main():
    args = parse_args()
    pairs = []
    for scene in args.scenes:
        left = cv2.imread(str(args.dataset / scene / "im0.png"), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(args.dataset / scene / "im1.png"), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            raise FileNotFoundError(scene)
        pairs.append((cv2.resize(left, SIZE, interpolation=cv2.INTER_AREA), cv2.resize(right, SIZE, interpolation=cv2.INTER_AREA)))

    failures = []
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
    for scene, pair in zip(args.scenes, pairs):
        actual, rectified_left, rectified_right = run_pair(pair, args.device, args.timeout)
        if not np.array_equal(rectified_left, pair[0]) or not np.array_equal(rectified_right, pair[1]):
            raise RuntimeError(f"{scene}: rectification-disabled passthrough changed the input")
        expected = cv2.imread(str(args.references / scene / "disparity.png"), cv2.IMREAD_UNCHANGED)
        if expected is None:
            expected = cv2.imread(str(args.references / scene / "disparity_0.png"), cv2.IMREAD_UNCHANGED)
        if expected is None:
            expected = cv2.imread(str(args.references / f"rvc2-default-1280-{scene}" / "disparity_0.png"), cv2.IMREAD_UNCHANGED)
        if expected is None:
            raise FileNotFoundError(f"{scene}: physical RVC2 reference not found")
        difference = actual.astype(np.int32) - expected.astype(np.int32)
        mismatches = int(np.count_nonzero(difference))
        print({
            "scene": scene,
            "shape": actual.shape,
            "mismatches": mismatches,
            "exact_percent": float(np.mean(difference == 0) * 100),
            "mae": float(np.mean(np.abs(difference))),
            "maximum_difference": int(np.max(np.abs(difference))),
        })
        if args.output:
            cv2.imwrite(str(args.output / f"{scene}.png"), actual)
        if mismatches:
            failures.append(scene)
    if failures:
        raise RuntimeError("RVC2 DEFAULT parity failed: " + ", ".join(failures))


if __name__ == "__main__":
    main()
