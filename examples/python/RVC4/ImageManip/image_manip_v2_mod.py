import depthai as dai
from time import sleep
import cv2

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
manip = pipeline.create(dai.node.ImageManipV2)

camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

manip.initialConfig.setOutputSize(1270, 710, dai.ImageManipConfigV2.ResizeMode.CENTER_CROP)
manip.initialConfig.rotateDeg(45)
manip.initialConfig.scale(0.5)
manip.initialConfig.crop(50, 100, 200, 200)
manip.initialConfig.flipVertical()
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888i)
manip.setMaxOutputFrameSize(2709360)

camRgb.video.link(manip.inputImage)

out = manip.out.createOutputQueue()

pipeline.start()

while True:
    inFrame = out.get()
    if inFrame is not None:
        cv2.imshow("HostDisplay", inFrame.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

pipeline.stop()

