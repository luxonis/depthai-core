import time
from pathlib import Path
from typing import Dict, Optional

import cv2
import depthai as dai
import numpy as np

TILE_SIZE = (640, 400)
RESEND_PERIOD_S = 0.5


class CaptureGallery(dai.node.ThreadedHostNode):
    """Collects one triggered capture from each peripheral and shows the batch until the next one."""

    def __init__(self) -> None:
        super().__init__()
        self.out = self.createOutput()
        self.captures: Dict[str, dai.Node.Input] = {}
        self.batch_count = 0
        self.save_dir: Optional[Path] = None

    def add_capture(self, label: str) -> dai.Node.Input:
        self.captures[label] = self.createInput(name=f"capture{len(self.captures)}", blocking=False, queueSize=2)
        return self.captures[label]

    def run(self) -> None:
        batch: Dict[str, dai.ImgFrame] = {}
        shown: Optional[dai.ImgFrame] = None
        last_sent = 0.0
        while self.mainLoop():
            for label, capture in self.captures.items():
                frame = capture.tryGet()
                if frame is not None:
                    batch[label] = frame
            if len(batch) == len(self.captures):
                self.batch_count += 1
                shown = self._compose(batch)
                print(f"Batch {self.batch_count}: " + ", ".join(f"{label} seq {f.getSequenceNum()}" for label, f in batch.items()), flush=True)
                if self.save_dir:
                    cv2.imwrite(str(self.save_dir / f"batch_{self.batch_count}.jpg"), shown.getCvFrame())
                batch = {}
                last_sent = 0.0
            if shown is not None and time.monotonic() - last_sent > RESEND_PERIOD_S:
                shown.setTimestamp(dai.Clock.now())
                self.out.send(shown)
                last_sent = time.monotonic()
            time.sleep(0.01)

    def _compose(self, batch: Dict[str, dai.ImgFrame]) -> dai.ImgFrame:
        tiles = []
        for label in self.captures:
            tile = cv2.resize(batch[label].getCvFrame(), TILE_SIZE)
            text = f"{label}  batch {self.batch_count}"
            cv2.putText(tile, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(tile, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
            tiles.append(tile)
        mosaic = dai.ImgFrame()
        mosaic.setCvFrame(np.ascontiguousarray(cv2.hconcat(tiles)), dai.ImgFrame.Type.BGR888i)
        return mosaic
