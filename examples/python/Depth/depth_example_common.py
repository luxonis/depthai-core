"""Shared helpers for dai.node.Depth Python examples."""

from __future__ import annotations

from typing import Tuple

import cv2
import depthai as dai
import numpy as np

# Fixed depth visualization range (millimeters). Values above this clamp to the max color.
DEPTH_VIS_MAX_MM = 15_000

_COLOR_MAP = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
_COLOR_MAP[0] = [0, 0, 0]


def colorizeDepthMm(frame: np.ndarray) -> np.ndarray:
    """Colorize RAW16 depth in millimeters using a fixed 0..15 m range (zero = invalid)."""
    valid = frame > 0
    if not np.any(valid):
        return np.zeros((*frame.shape, 3), dtype=np.uint8)

    norm = np.zeros_like(frame, dtype=np.uint8)
    clipped = np.minimum(frame[valid].astype(np.float32), float(DEPTH_VIS_MAX_MM))
    norm[valid] = (clipped / DEPTH_VIS_MAX_MM * 255.0).astype(np.uint8)

    colored = cv2.applyColorMap(norm, _COLOR_MAP)
    colored[~valid] = 0
    return colored


def colorizeConfidence(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint16:
        vmax = int(np.max(frame))
        if vmax <= 0:
            return np.zeros((*frame.shape, 3), dtype=np.uint8)
        vis = ((frame.astype(np.float32) / vmax) * 255).astype(np.uint8)
    else:
        vis = frame
    return cv2.applyColorMap(vis, _COLOR_MAP)


def create_depth_output_queues(
    depth_node: dai.node.Depth,
) -> Tuple[dai.MessageQueue, dai.MessageQueue]:
    depth_queue = depth_node.depth.createOutputQueue()
    confidence_queue = depth_node.confidence.createOutputQueue()
    return depth_queue, confidence_queue


def require_default_device(pipeline: dai.Pipeline) -> dai.Device:
    device = pipeline.getDefaultDevice()
    if device is None:
        raise RuntimeError("Connect a device (host-only pipeline cannot use Depth).")
    return device


def require_first_stereo_pair(device: dai.Device) -> dai.StereoPair:
    pairs = device.getStereoPairs()
    if not pairs:
        raise RuntimeError("This device has no stereo pair; Depth cannot run.")
    return pairs[0]


def find_stereo_depth_in_pipeline(pipeline: dai.Pipeline) -> dai.node.StereoDepth | None:
    """Return the ``StereoDepth`` node created inside a ``Depth`` STEREO backend, if present."""
    for node in pipeline.getAllNodes():
        if isinstance(node, dai.node.StereoDepth):
            return node
    return None


def device_has_tof_sensor(device: dai.Device) -> bool:
    try:
        for cf in device.getConnectedCameraFeatures():
            for sensor_type in cf.supportedTypes:
                if sensor_type == dai.CameraSensorType.TOF:
                    return True
    except Exception:
        return False
    return False


def find_color_camera_socket(device: dai.Device, stereo_pair: dai.StereoPair) -> dai.CameraBoardSocket | None:
    """Return a connected color/RGB socket outside the stereo pair, if any."""
    stereo_sockets = {stereo_pair.left, stereo_pair.right}
    try:
        for cf in device.getConnectedCameraFeatures():
            if cf.socket in stereo_sockets or cf.socket == dai.CameraBoardSocket.AUTO:
                continue
            for sensor_type in cf.supportedTypes:
                if sensor_type == dai.CameraSensorType.COLOR:
                    return cf.socket
    except Exception:
        pass

    try:
        connected = set(device.getConnectedCameras())
        if dai.CameraBoardSocket.CAM_A in connected and dai.CameraBoardSocket.CAM_A not in stereo_sockets:
            return dai.CameraBoardSocket.CAM_A
    except Exception:
        pass
    return None
