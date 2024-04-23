import depthai as dai
import cv2
import time


class HostDisplay(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.input = dai.Node.Input(self)
    def run(self):
        while True:
            message : dai.ImgFrame = self.input.get()
            cv2.imshow("HostDisplay", message.getCvFrame())
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.stopPipeline()
                break

with dai.Pipeline() as p:
    camera = p.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    myHostDisplay = p.create(HostDisplay)
    camera.video.link(myHostDisplay.input)

    p.start()
    while p.isRunning():
        time.sleep(1)