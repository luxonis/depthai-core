import depthai as dai
import cv2

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
manip = pipeline.create(dai.node.ImageManip)

manip.initialConfig.setOutputSize(300, 300, dai.ImageManipConfig.ResizeMode.STRETCH)

camOut = camRgb.requestOutput((1920, 1080))
camOut.link(manip.inputImage)

manipQ = manip.out.createOutputQueue()
camQ = camOut.createOutputQueue()

pipeline.start()

while True:
    if manipQ.has():
        cv2.imshow("Manip frame", manipQ.get().getCvFrame())
    if camQ.has():
        cv2.imshow("Camera frame", camQ.get().getCvFrame())
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
