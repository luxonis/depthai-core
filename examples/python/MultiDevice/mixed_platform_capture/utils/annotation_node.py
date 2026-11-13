import math
from typing import List, Tuple

import depthai as dai
from depthai.beta import Keypoints, Predictions
from depthai_nodes import SECONDARY_COLOR, GatheredData
from depthai_nodes.utils import AnnotationHelper

from utils.gesture_recognition import recognize_gesture
from utils.process import hand_region

FIST_FRAMES = 5  # frames in a row with a fist before the trigger fires
OPEN_FRAMES = 15  # frames in a row without a fist before the next trigger can fire


class AnnotationNode(dai.node.HostNode):
    """Draws the hands, labels their gesture and sends a trigger when a fist appears."""

    def __init__(self) -> None:
        super().__init__()
        self.out_pose_annotations = self.createOutput(possibleDatatypes=[dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgAnnotations, True)])
        self.out_fist = self.createOutput()
        self.confidence_threshold = 0.5
        self.connection_pairs: List[List[int]] = []
        self.armed = True
        self.streak = 0

    def build(self, gathered_data: dai.Node.Output, confidence_threshold: float, connections_pairs: List[List[int]]) -> "AnnotationNode":
        self.confidence_threshold = confidence_threshold
        self.connection_pairs = connections_pairs
        self.link_args(gathered_data)
        return self

    def process(self, gathered_data: dai.Buffer) -> None:
        assert isinstance(gathered_data, GatheredData)
        detections: dai.ImgDetections = gathered_data.reference_data
        annotations = AnnotationHelper()
        fist_shown = False

        for ix, detection in enumerate(detections.detections):
            keypoints_msg: Keypoints = gathered_data.items[ix]["0"]
            confidence_msg: Predictions = gathered_data.items[ix]["1"]
            handedness_msg: Predictions = gathered_data.items[ix]["2"]
            if confidence_msg.getFirstPrediction() < self.confidence_threshold:
                continue

            rect = hand_region(detection.getBoundingBox())
            points = [to_frame(rect, kp.imageCoordinates.x, kp.imageCoordinates.y) for kp in keypoints_msg.getKeypoints()]
            gesture = recognize_gesture(points)
            fist_shown |= gesture == "FIST"

            color = dai.Color(1.0, 0.2, 0.2, 1.0) if gesture == "FIST" else SECONDARY_COLOR
            for a, b in self.connection_pairs:
                annotations.draw_line(pt1=points[a], pt2=points[b], color=color, thickness=2)
            annotations.draw_points(points=points, color=color, thickness=4)
            hand = "Left" if handedness_msg.getFirstPrediction() < 0.5 else "Right"
            label_y = max(rect.center.y - rect.size.height / 2 - 0.03, 0.02)
            annotations.draw_text(text=f"{hand} {gesture}", position=(rect.center.x - 0.06, label_y), color=color, size=28)

        self.out_pose_annotations.send(annotations.build(timestamp=detections.getTimestamp(), sequence_num=detections.getSequenceNum()))
        if self._fist_started(fist_shown):
            trigger = dai.Buffer()
            trigger.setTimestamp(detections.getTimestamp())
            trigger.setSequenceNum(detections.getSequenceNum())
            self.out_fist.send(trigger)

    def _fist_started(self, fist_shown: bool) -> bool:
        """Armed: count fist frames and fire at FIST_FRAMES. Fired: count frames without a fist and re-arm at OPEN_FRAMES."""
        self.streak = self.streak + 1 if fist_shown == self.armed else 0
        if self.armed and self.streak >= FIST_FRAMES:
            self.armed, self.streak = False, 0
            return True
        if not self.armed and self.streak >= OPEN_FRAMES:
            self.armed, self.streak = True, 0
        return False


def to_frame(rect: dai.RotatedRect, u: float, v: float) -> Tuple[float, float]:
    """Map a keypoint from the rotated hand crop (0..1) back to the normalized frame.

    A keypoint out of the frame (for example 1.02) is kept as it is: it is still normalized.
    """
    dx = (u - 0.5) * rect.size.width
    dy = (v - 0.5) * rect.size.height
    angle = math.radians(rect.angle)
    x = rect.center.x + dx * math.cos(angle) - dy * math.sin(angle)
    y = rect.center.y + dx * math.sin(angle) + dy * math.cos(angle)
    return (x, y)
