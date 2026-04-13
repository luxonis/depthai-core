#!/usr/bin/env python3
"""
Same flow as unified_depth.py, but left/right Camera nodes are created explicitly on the device's
first stereo pair before Depth.build(). Depth then reuses those cameras (ISP at 30 FPS) instead
of creating and adopting its own.

You can also create(dai.node.Depth) first, then the two Camera nodes, then depth.build(); both orders work.
"""

import sys

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
device = pipeline.getDefaultDevice()
if device is None:
    print("Connect a device (host-only pipeline cannot use Depth).", file=sys.stderr)
    sys.exit(1)

pairs = device.getStereoPairs()
if not pairs:
    print("This device has no stereo pair; Depth cannot run.", file=sys.stderr)
    sys.exit(1)

sp = pairs[0]

pipeline.create(dai.node.Camera).build(sp.left, sensorResolution=(1280, 800), sensorFps=30)
pipeline.create(dai.node.Camera).build(sp.right, sensorResolution=(1280, 800), sensorFps=30)

depth_node = pipeline.create(dai.node.Depth)
depth_node.build()

stereo = depth_node.getStereoDepth()
if stereo is not None:
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

depth_queue = depth_node.depth.createOutputQueue()
confidence_queue = depth_node.confidence.createOutputQueue()

color_map = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
color_map[0] = [0, 0, 0]


def colorize_depth_mm(frame: np.ndarray) -> np.ndarray:
    valid = frame > 0
    if not np.any(valid):
        return np.zeros((*frame.shape, 3), dtype=np.uint8)
    vmax = float(np.max(frame[valid]))
    norm = np.zeros_like(frame, dtype=np.uint8)
    norm[valid] = ((frame[valid].astype(np.float32) / vmax) * 255).astype(np.uint8)
    return cv2.applyColorMap(norm, color_map)


def colorize_confidence(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint16:
        vmax = int(np.max(frame))
        if vmax <= 0:
            return np.zeros((*frame.shape, 3), dtype=np.uint8)
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8)
    else:
        vis = frame
    return cv2.applyColorMap(vis, color_map)


with pipeline:
    pipeline.start()
    while pipeline.isRunning():
        depth_frame = depth_queue.get()
        assert isinstance(depth_frame, dai.ImgFrame)
        cv2.imshow("depth (user cameras)", colorize_depth_mm(depth_frame.getFrame()))

        conf_frame = confidence_queue.get()
        assert isinstance(conf_frame, dai.ImgFrame)
        cv2.imshow("confidence", colorize_confidence(conf_frame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break
