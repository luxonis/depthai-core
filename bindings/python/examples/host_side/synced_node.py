import depthai as dai
import numpy as np
import cv2

class SyncedDisplay(dai.SyncedNode):
    def __init__(self):
        dai.SyncedNode.__init__(self)
        self.inputMono = self.inputs["mono"]
        self.inputRgb = self.inputs["rgb"]

    def runOnce(self, messageGroup):
        frame = np.empty((640, 2*640, 3), dtype=np.uint8)
        frame[:, :640, :] = cv2.resize(messageGroup["mono"].getCvFrame(), (640, 640)).reshape((640, 640, 1))
        frame[:, 640:, :] = cv2.resize(messageGroup["rgb"].getCvFrame(), (640, 640))
        cv2.imshow("synced", frame)
        if cv2.waitKey(1) == ord("q"): self.stopPipeline()



p = dai.Pipeline(True)
color_camera = p.create(dai.node.ColorCamera)
mono_camera = p.create(dai.node.MonoCamera)
display = SyncedDisplay()
p.add(display)
color_camera.video.link(display.inputRgb)
mono_camera.out.link(display.inputMono)
p.start()
p.wait()
