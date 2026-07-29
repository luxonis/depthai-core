"""Shared visualization / debug helpers for the focused-depth deployment examples.

Kept in one place so the three examples (roi / detection / budget) render the focused
depth identically. Visualization is optional: it is skipped automatically when there is no
display (e.g. running headless over SSH) or when OpenCV is unavailable.
"""

import os

import numpy as np

try:
    import cv2
except ImportError:  # OpenCV is optional; examples still run headless without it.
    cv2 = None


def display_enabled(headless):
    """True only if a window can actually be shown (OpenCV present and a display attached)."""
    return (not headless) and cv2 is not None and bool(os.environ.get("DISPLAY"))


def colorize_depth(depth_mm, max_mm):
    """Colorize a RAW16 depth map (mm). Invalid/clipped pixels stay black."""
    depth = depth_mm.astype(np.float32)
    if max_mm:
        depth[depth > max_mm] = 0.0
    valid = depth > 0
    canvas = np.zeros((depth.shape[0], depth.shape[1], 3), np.uint8)
    if valid.any() and cv2 is not None:
        low = float(depth[valid].min())
        high = float(depth[valid].max())
        norm = np.zeros_like(depth)
        norm[valid] = (depth[valid] - low) / max(1.0, high - low)
        canvas = cv2.applyColorMap((norm * 255.0).astype(np.uint8), cv2.COLORMAP_JET)
        canvas[~valid] = 0
    return canvas


def read_debug(debug_queue, block=False):
    """Return the focusDebug trace string. focusDebug is emitted once per focused-depth frame,
    so block=True keeps it aligned 1:1 with the depth output; block=False returns '' if none is
    pending."""
    if debug_queue is None:
        return ""
    message = debug_queue.get() if block else debug_queue.tryGet()
    if message is None:
        return ""
    return bytes(message.getData()).decode("utf-8", "replace")


def close_windows():
    """Tear down any OpenCV windows before the pipeline closes. No-op when OpenCV has no GUI
    support (e.g. opencv-python-headless), where destroyAllWindows raises cv2.error."""
    if cv2 is None:
        return
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass


def show(window, depth_mm, max_mm, lines, headless):
    """Show the colorized depth with overlay text. Returns False when the user asks to quit."""
    if not display_enabled(headless):
        return True
    canvas = colorize_depth(depth_mm, max_mm)
    y = 18
    for line in lines:
        cv2.putText(canvas, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        y += 18
    try:
        cv2.imshow(window, canvas)
        return cv2.waitKey(1) != ord("q")
    except cv2.error:
        # OpenCV without GUI support (opencv-python-headless): carry on without a window.
        return True
