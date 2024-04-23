import depthai as dai
import numpy as np
import cv2

class Display(dai.node.HostNode):
    def process(self, rgb: dai.ImgFrame, mono: dai.ImgFrame):
        frame = np.empty((640, 2*640, 3), dtype=np.uint8)
        frame[:, :640, :] = cv2.resize(mono.getCvFrame(), (640, 640)).reshape((640, 640, 1))
        frame[:, 640:, :] = cv2.resize(rgb.getCvFrame(), (640, 640))
        cv2.imshow("Display", frame)
        if cv2.waitKey(1) == ord("q"):
            self.stopPipeline()

with dai.Pipeline(createImplicitDevice=True) as p:
    colorCamera = p.create(dai.node.ColorCamera)
    colorCamera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    monoCamera = p.create(dai.node.MonoCamera)
    monoCamera.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    display = p.create(Display).build(colorCamera.video, monoCamera.out)
    p.start()
    p.wait()
