import depthai as dai
import cv2

pipeline = dai.Pipeline()
device = pipeline.getDefaultDevice()

camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
manip = pipeline.create(dai.node.ImageManip)

# GPU is not available on RVC2 and some RVC4 devices
manip.setBackend(dai.node.ImageManip.Backend.GPU if device.hasGPU() else
                 dai.node.ImageManip.Backend.CPU)

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
