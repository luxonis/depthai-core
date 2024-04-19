import depthai as dai
import numpy as np
import cv2


class Display(dai.node.HostNode):
    def __init__(self):
        dai.node.HostNode.__init__(self) # Always needs to be called
        self.inputMono = self.inputs["mono"]
        self.inputRgb = self.inputs["rgb"]
        self.name = ""

    def build(self, outputRgb: dai.Node.Output, outputMono: dai.Node.Output, name="Display"):
        outputRgb.link(self.inputMono)
        outputMono.link(self.inputRgb)
        self.name = name
        return self

    def runOnce(self, messages):
        frame = np.empty((640, 2*640, 3), dtype=np.uint8)
        frame[:, :640, :] = cv2.resize(messages["mono"].getCvFrame(), (640, 640)).reshape((640, 640, 1))
        frame[:, 640:, :] = cv2.resize(messages["rgb"].getCvFrame(), (640, 640))
        cv2.imshow(self.name, frame)
        if cv2.waitKey(1) == ord("q"):
            self.stopPipeline()


with dai.Pipeline(createImplicitDevice=True) as p:
    color_camera = p.create(dai.node.ColorCamera)
    color_camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    mono_camera = p.create(dai.node.MonoCamera)
    mono_camera.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    display = p.create(Display).build(color_camera.video, mono_camera.out)
    p.start()
    p.wait()
