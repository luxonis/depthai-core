import math
from typing import Tuple

import depthai as dai

# MediaPipe hand region: the palm box scaled up and moved toward the fingers, in palm sizes.
HAND_SCALE = 2.6
HAND_SHIFT = 0.5


def hand_region(palm: dai.RotatedRect) -> dai.RotatedRect:
    """The square, rotated region around the whole hand that the hand landmark model expects.

    Near the frame edge the region goes out of the frame (center < 0 or > 1, side > 1). These
    values are still normalized, so set the flag: without it depthai guesses from the values.
    """
    side = max(palm.size.width, palm.size.height)
    angle = math.radians(palm.angle)
    shift = HAND_SHIFT * palm.size.height
    center = dai.Point2f(x=palm.center.x + shift * math.sin(angle), y=palm.center.y - shift * math.cos(angle), normalized=True)
    size = dai.Size2f(width=side * HAND_SCALE, height=side * HAND_SCALE, normalized=True)
    return dai.RotatedRect(center, size, palm.angle)


class ProcessDetections(dai.node.HostNode):
    """Turns palm detections into one ImageManip crop config per hand.

    Each crop is the rotated hand region, so the hand landmark model gets an upright hand.
    """

    def __init__(self) -> None:
        super().__init__()
        self.detections_input = self.createInput()
        self.config_output = self.createOutput()
        self._target_size = (224, 224)

    def build(self, detections_input: dai.Node.Output, target_size: Tuple[int, int]) -> "ProcessDetections":
        self._target_size = target_size
        self.link_args(detections_input)
        return self

    def process(self, img_detections: dai.Buffer) -> None:
        assert isinstance(img_detections, dai.ImgDetections)
        configs_group = dai.MessageGroup()
        for i, detection in enumerate(img_detections.detections):
            cfg = dai.ImageManipConfig()
            cfg.addCropRotatedRect(hand_region(detection.getBoundingBox()), normalizedCoords=True)
            cfg.setOutputSize(*self._target_size, dai.ImageManipConfig.ResizeMode.STRETCH)
            cfg.setReusePreviousImage(False)
            cfg.setTimestamp(img_detections.getTimestamp())
            cfg.setSequenceNum(img_detections.getSequenceNum())
            configs_group[f"cfg_{i}"] = cfg

        configs_group.setTimestamp(img_detections.getTimestamp())
        # FrameCropper syncs this group with the frame on the device, by device timestamp
        configs_group.setTimestampDevice(img_detections.getTimestampDevice())
        configs_group.setSequenceNum(img_detections.getSequenceNum())
        self.config_output.send(configs_group)
