import depthai as dai
from time import sleep

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
manip = pipeline.create(dai.node.ImageManipV2)
display = pipeline.create(dai.node.Display)

camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

manip.initialConfig.setOutputSize(400, 400, dai.ImageManipV2Config.ResizeMode.CENTER_CROP)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888i)
manip.setMaxOutputFrameSize(2709360)

camRgb.video.link(manip.inputImage)
manip.out.link(display.input)

pipeline.start()

sleep(30)

pipeline.stop()

