#!/usr/bin/env python3
"""Focused-depth detector deployment example (largest detection only)."""

import argparse
import time

import depthai as dai
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="yolov6-nano")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detector = pipeline.create(dai.node.DetectionNetwork).build(camera, dai.NNModelDescription(args.model))
    detector.setConfidenceThreshold(0.5)
    depth = pipeline.create(dai.node.Depth)
    depth.setFocusModels([dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM])
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.LARGEST)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.SINGLE_TIER_PER_FRAME)
    depth.build(args.fps)
    detector.out.link(depth.inputDetections)
    detections = detector.out.createOutputQueue(maxSize=4, blocking=False)
    output = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)

    with pipeline:
        pipeline.start()
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print(f"Focus mode: detector / LARGEST / SINGLE_TIER_PER_FRAME / model={args.model}")
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
