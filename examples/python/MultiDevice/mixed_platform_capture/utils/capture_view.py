import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import depthai as dai
import numpy as np

RESEND_PERIOD_S = 1.0


class CaptureView(dai.node.ThreadedHostNode):
    """One viewer stream for one capture camera: black until the first capture, then the last capture until the next one."""

    def __init__(self) -> None:
        super().__init__()
        self.capture = self.createInput(blocking=False, queueSize=2)
        self.out = self.createOutput()
        self.size = (1280, 800)
        self.name = ""
        self.save_dir: Optional[Path] = None
        self.capture_count = 0

    def build(self, capture: dai.Node.Output, size: Tuple[int, int], name: str, save_dir: Optional[Path]) -> "CaptureView":
        capture.link(self.capture)
        self.size, self.name, self.save_dir = size, name, save_dir
        return self

    def run(self) -> None:
        shown = dai.ImgFrame()
        shown.setCvFrame(np.zeros((self.size[1], self.size[0], 3), np.uint8), dai.ImgFrame.Type.NV12)
        last_sent = 0.0
        while self.mainLoop():
            frame = self.capture.tryGet()
            if frame is not None:
                self.capture_count += 1
                shown = frame
                last_sent = 0.0
                print(f"Capture {self.capture_count} from {self.name}: {frame.getWidth()}x{frame.getHeight()} seq {frame.getSequenceNum()}", flush=True)
                if self.save_dir:
                    cv2.imwrite(str(self.save_dir / f"{self.name}_{self.capture_count}.jpg"), frame.getCvFrame())
            if time.monotonic() - last_sent > RESEND_PERIOD_S:
                shown.setTimestamp(dai.Clock.now())
                self.out.send(shown)
                last_sent = time.monotonic()
            time.sleep(0.01)
