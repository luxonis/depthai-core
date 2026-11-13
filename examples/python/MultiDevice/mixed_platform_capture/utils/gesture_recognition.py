from typing import Sequence, Tuple

import numpy as np

WRIST = 0
# (PIP, TIP) keypoint indices of the MediaPipe hand landmarks, thumb excluded.
FINGERS = {"index": (6, 8), "middle": (10, 12), "ring": (14, 16), "pinky": (18, 20)}
EXTENDED_RATIO = 1.15


def recognize_gesture(keypoints: Sequence[Tuple[float, float]]) -> str:
    """Classify a hand from its 21 keypoints (in a square frame) by how many fingers are curled.

    A finger is curled when its tip is closer to the wrist than its PIP joint, and extended when
    the tip is clearly farther. The test uses distances only, so it works at any hand rotation.
    """
    points = np.asarray(keypoints, dtype=np.float32)
    wrist = points[WRIST]
    curled = extended = 0
    for pip, tip in FINGERS.values():
        tip_distance = np.linalg.norm(points[tip] - wrist)
        pip_distance = np.linalg.norm(points[pip] - wrist)
        if tip_distance < pip_distance:
            curled += 1
        elif tip_distance > EXTENDED_RATIO * pip_distance:
            extended += 1
    if curled == len(FINGERS):
        return "FIST"
    if extended == len(FINGERS):
        return "OPEN"
    return f"{extended} FINGERS"
