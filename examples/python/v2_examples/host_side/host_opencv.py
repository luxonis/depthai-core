import depthai as dai
import cv2
import time

p = dai.Pipeline(True)

class HostDisplay(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
    def run(self):
        while True:
            message : dai.ImgFrame = self.input.get()
            cv2.imshow("HostDisplay", message.getCvFrame())
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.stopPipeline()
                break

camera = p.create(dai.node.ColorCamera)
camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)

myHostDisplay = p.create(HostDisplay)
camera.video.link(myHostDisplay.input)

p.start()
p.wait()