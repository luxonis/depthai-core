import depthai as dai
import cv2

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
manip = pipeline.create(dai.node.ImageManipV2)


manip.initialConfig.setOutputSize(1270, 710, dai.ImageManipConfigV2.ResizeMode.LETTERBOX)
manip.initialConfig.addCrop(50, 100, 500, 500)
manip.initialConfig.addFlipVertical()
manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
manip.setMaxOutputFrameSize(2709360)

camOut = camRgb.requestOutput((1920, 1080))
camOut.link(manip.inputImage)

manipQ = manip.out.createOutputQueue()
camQ = camOut.createOutputQueue()

pipeline.start()

print(manip.initialConfig)

while True:
    if manipQ.has():
        cv2.imshow("Manip frame", manipQ.get().getCvFrame())
    if camQ.has():
        cv2.imshow("Camera frame", camQ.get().getCvFrame())
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
