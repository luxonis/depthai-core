import depthai as dai
import cv2
import time

p = dai.Pipeline()

class HostDisplay(dai.HostNode):
    def __init__(self):
        dai.HostNode.__init__(self)
        self.input = dai.Node.Input(self)
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

myHostDisplay = HostDisplay()
p.add(myHostDisplay)
camera.video.link(myHostDisplay.input)

p.start()
p.wait()