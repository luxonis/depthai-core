#!/usr/bin/env python3
"""Focused-depth ROI deployment example (host-published detections)."""

import argparse
import threading
import time

import depthai as dai
import numpy as np


def detections(roi):
    det = dai.ImgDetection()
    det.label = 0
    det.confidence = 1.0
    det.xmin, det.ymin, det.xmax, det.ymax = roi
    message = dai.ImgDetections()
    message.detections = [det]
    message.setTimestamp(dai.Clock.now())
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--roi", default="0.35,0.30,0.65,0.70")
    args = parser.parse_args()
    roi = tuple(float(value) for value in args.roi.split(","))
    if len(roi) != 4 or not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1):
        parser.error("--roi must be xmin,ymin,xmax,ymax in normalized coordinates")

    pipeline = dai.Pipeline()
    depth = pipeline.create(dai.node.Depth)
    depth.setFocusModels([dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM])
    depth.setFocusSelectionMode(dai.node.Depth.FocusSelectionMode.ALL)
    depth.setFocusDispatchMode(dai.node.Depth.FocusDispatchMode.SINGLE_TIER_PER_FRAME)
    depth.build(args.fps)
    roi_queue = depth.inputDetections.createInputQueue()
    output_queue = depth.focusedDepth.createOutputQueue(maxSize=4, blocking=False)

    stop_event = threading.Event()

    def publish_roi():
        while not stop_event.is_set():
            try:
                roi_queue.send(detections(roi))
            except Exception:
                return
            stop_event.wait(0.015)

    with pipeline:
        pipeline.start()
        print("Resolved algorithm:", depth.getResolvedAlgorithm())
        print("Focus mode: ROI / ALL / SINGLE_TIER_PER_FRAME / model=NEURAL_DEPTH_MEDIUM")
        publisher = threading.Thread(target=publish_roi, daemon=True)
        publisher.start()
        try:
            start = time.monotonic()
            for index in range(args.frames):
                frame = output_queue.get()
                elapsed = time.monotonic() - start
                valid = int(np.count_nonzero(frame.getFrame()))
                print(f"frame={index:03d} valid={valid} time_ms={elapsed * 1000:.1f} fps={(index + 1) / elapsed:.2f}")
        finally:
            stop_event.set()
            publisher.join(timeout=2)


if __name__ == "__main__":
    main()
