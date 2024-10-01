import depthai as dai
import cv2

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
manip = pipeline.create(dai.node.ImageManipV2)


manip.initialConfig.setOutputSize(300, 300, dai.ImageManipConfigV2.ResizeMode.STRETCH)

camRgb.requestOutput((1920, 1080)).link(manip.inputImage)

out = manip.out.createOutputQueue()

pipeline.start()

while True:
    inFrame = out.get()
    if inFrame is not None:
        cv2.imshow("Show frame", inFrame.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
