import depthai as dai
import cv2
import time


class HostCamera(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        # Create a VideoCapture object
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            p.stop()
            raise RuntimeError("Error: Couldn't open host camera")
        while self.isRunning():
            # Read the frame from the camera
            ret, frame = cap.read()
            if not ret:
                break
            # Create an ImgFrame message
            imgFrame = dai.ImgFrame()
            imgFrame.setData(frame)
            imgFrame.setWidth(frame.shape[1])
            imgFrame.setHeight(frame.shape[0])
            imgFrame.setType(dai.ImgFrame.Type.RGB888i)
            # Send the message
            self.output.send(imgFrame)
            # Wait for the next frame
            time.sleep(0.1)

with dai.Pipeline() as p:
    hostCamera = p.create(HostCamera)
    camQueue = hostCamera.output.createQueue()

    p.start()
    while p.isRunning():
        image : dai.ImgFrame = camQueue.get()
        cv2.imshow("HostCamera", image.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            p.stop()
            break