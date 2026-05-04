#!/usr/bin/env python3
"""
Stereo-style demo using the unified dai.node.Depth group (StereoDepth on RVC2/RVC3, NeuralDepth on RVC4).

- On RVC4 the NeuralDepth zoo model is fixed inside the library (not set via ``build()``).
- Create host queues on ``depth_node.depth`` / ``confidence`` **before** ``pipeline.build()`` (the pipeline
  rejects new queues after build). The first ``depth``/``confidence`` access wires cameras and the backend.
- For a non-``AUTO`` backend, pass ``dai.node.Depth.Algorithm`` to ``pipeline.create`` (e.g.
  ``pipeline.create(dai.node.Depth, dai.node.Depth.Algorithm.TOF)``), or ``algorithm=`` as a keyword.
"""

import cv2
import depthai as dai
import numpy as np

pipeline = dai.Pipeline()
depth_node = pipeline.create(dai.node.Depth)

depth_queue = depth_node.depth.createOutputQueue()
confidence_queue = depth_node.confidence.createOutputQueue()

pipeline.build()

color_map = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
color_map[0] = [0, 0, 0]  # invalid / zero depth pixels stay black


def colorize_depth_mm(frame: np.ndarray) -> np.ndarray:
    """RAW16 depth in millimeters; zero means invalid."""
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
        cv2.imshow("depth", colorize_depth_mm(depth_frame.getFrame()))

        conf_frame = confidence_queue.get()
        assert isinstance(conf_frame, dai.ImgFrame)
        cv2.imshow("confidence", colorize_confidence(conf_frame.getFrame()))

        key = cv2.waitKey(1)
        if key == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
