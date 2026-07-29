#!/usr/bin/env python3
"""Focused-depth detector deployment example with a per-frame time budget.

Crops are processed largest-first within a budget of one frame period (1000/fps ms).
For each crop the controller picks the smallest Nano/S/M model that fits both the crop
size and the remaining budget (downgrading toward Nano when time is short), using
per-model cost estimates fitted to the on-device benchmark in luxonis/depthai-core#1912
(Nano ~49 ms, S ~57 ms, M ~68 ms per inference). Remaining crops are skipped once the
budget is exhausted, so the node stays within the frame period.
"""

import argparse
import time

import depthai as dai
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov6-nano")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=30.0, help="Target focused-depth FPS budget")
    args = parser.parse_args()

    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detector = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
    detector.setConfidenceThreshold(0.5)
    depth = pipeline.create(dai.node.Depth)
    depth.setFocusModels(
        [
            dai.DeviceModelZoo.NEURAL_DEPTH_NANO,
            dai.DeviceModelZoo.NEURAL_DEPTH_SMALL,
            dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM,
        ]
    )
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.ALL)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.TIME_BUDGET)
    depth.build(args.fps)
    detector.out.link(depth.inputDetections)
    detections = detector.out.createOutputQueue(maxSize=4, blocking=False)
    output = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)

    with pipeline:
        pipeline.start()
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print(f"Focus mode: detector / ALL / TIME_BUDGET / target_fps={args.fps:.1f} / models=NANO,SMALL,MEDIUM")
        start = time.monotonic()
        for index in range(args.frames):
            frame = output.get()
            detection_message = detections.tryGet()
            count = len(detection_message.detections) if detection_message else 0
            elapsed = time.monotonic() - start
            valid = int(np.count_nonzero(frame.getFrame()))
            print(f"frame={index:03d} detections={count} valid={valid} time_ms={elapsed * 1000:.1f} fps={(index + 1) / elapsed:.2f}")


if __name__ == "__main__":
    main()
