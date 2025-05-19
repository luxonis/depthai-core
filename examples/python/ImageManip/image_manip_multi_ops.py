import depthai as dai
import cv2

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
manip = pipeline.create(dai.node.ImageManip)


manip.initialConfig.setOutputSize(1270, 710, dai.ImageManipConfig.ResizeMode.LETTERBOX)
manip.initialConfig.addCrop(50, 100, 500, 500)
manip.initialConfig.addFlipVertical()
manip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
manip.setMaxOutputFrameSize(2709360)

camRgb.requestOutput((1920, 1080)).link(manip.inputImage)

out = manip.out.createOutputQueue()

pipeline.start()

print(manip.initialConfig)

while True:
    inFrame = out.get()
    if inFrame is not None:
        cv2.imshow("Show frame", inFrame.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
