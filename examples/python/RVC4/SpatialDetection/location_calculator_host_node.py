#!/usr/bin/env python3

import depthai as dai
import cv2
import time
import numpy as np
from pathlib import Path

datasetDefault = str((Path(__file__).parent / Path('../../models/dataset')).resolve().absolute())
if not Path(datasetDefault).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

depthImg = cv2.imread(datasetDefault+"/DFS_DispMap.pgm", cv2.IMREAD_GRAYSCALE)

width = 1280
height = 720
color = (255, 255, 255)

data = depthImg.reshape(height*width)
doubleData = np.zeros(height*width*2)
doubleData = (doubleData).astype('uint8')

for i in range(height*width):
    temp = data[i] & 0xFFFF

    doubleData[i*2+1]= temp >> 8
    doubleData[i*2] = temp & 0xFF

cv2.imshow("depthImg", depthImg)
cv2.waitKey(10)

class HostNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        while self.isRunning():
            frame = depthImg

            imgFrame = dai.ImgFrame()
            imgFrame.setData(doubleData)
            imgFrame.setWidth(frame.shape[1])
            imgFrame.setHeight(frame.shape[0])
            imgFrame.setType(dai.ImgFrame.Type.RAW16)
            # Send the message
            self.output.send(imgFrame)
            # Wait for the next frame
            time.sleep(0.1)

# Create pipeline
info = dai.DeviceInfo("127.0.0.1")
info.protocol = dai.X_LINK_TCP_IP
info.state = dai.X_LINK_GATE
info.platform = dai.X_LINK_RVC3

with dai.Device(info) as device:
    with dai.Pipeline(device) as pipeline:
        cnt = 0
        hostNode = pipeline.create(HostNode)
        camQueue = hostNode.output.createOutputQueue()
        
        # Config
        topLeft = dai.Point2f(0.35, 0.35)
        bottomRight = dai.Point2f(0.38, 0.38)

        config = dai.SpatialLocationCalculatorConfigData()
        config.depthThresholds.lowerThreshold = 10
        config.depthThresholds.upperThreshold = 10000
        calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
        config.roi = dai.Rect(topLeft, bottomRight)

        spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
        spatialLocationCalculator.inputConfig.setWaitForMessage(False)
        spatialLocationCalculator.initialConfig.addROI(config)


        xoutSpatialQueue = spatialLocationCalculator.out.createOutputQueue()
        outputDepthQueue = spatialLocationCalculator.passthroughDepth.createOutputQueue()


        hostNode.output.link(spatialLocationCalculator.inputDepth)

        inputConfigQueue = spatialLocationCalculator.inputConfig.createInputQueue()

        pipeline.start()
        while pipeline.isRunning():
            spatialData = xoutSpatialQueue.get().getSpatialLocations()

            image : dai.ImgFrame = camQueue.get()
            outputDepthIMage : dai.ImgFrame = outputDepthQueue.get()

            frameDepth = outputDepthIMage.getCvFrame()

            depthFrameColor = cv2.normalize(frameDepth, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
            for depthData in spatialData:
                roi = depthData.config.roi
                roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                depthMin = depthData.depthMin
                depthMax = depthData.depthMax

                fontType = cv2.FONT_HERSHEY_TRIPLEX
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
                cv2.putText(depthFrameColor, f"X: {int(depthData.spatialCoordinates.x)} mm", (xmin + 10, ymin + 20), fontType, 0.5, 255)
                cv2.putText(depthFrameColor, f"Y: {int(depthData.spatialCoordinates.y)} mm", (xmin + 10, ymin + 35), fontType, 0.5, 255)
                cv2.putText(depthFrameColor, f"Z: {int(depthData.spatialCoordinates.z)} mm", (xmin + 10, ymin + 50), fontType, 0.5, 255)
            # Show the frame
            cv2.imshow("disp", depthFrameColor)

            key = cv2.waitKey(1)
            if key == ord('q'):
                pipeline.stop()
                break

            cnt += 1
            stepSize = 0.05

            newConfig = False

            key = cv2.waitKey(1000)
            if key == ord('q'):
                break
            elif key == ord('w'):
                if topLeft.y - stepSize >= 0:
                    topLeft.y -= stepSize
                    bottomRight.y -= stepSize
                    newConfig = True
            elif key == ord('a'):
                if topLeft.x - stepSize >= 0:
                    topLeft.x -= stepSize
                    bottomRight.x -= stepSize
                    newConfig = True
            elif key == ord('s'):
                if bottomRight.y + stepSize <= 1:
                    topLeft.y += stepSize
                    bottomRight.y += stepSize
                    newConfig = True
            elif key == ord('d'):
                if bottomRight.x + stepSize <= 1:
                    topLeft.x += stepSize
                    bottomRight.x += stepSize
                    newConfig = True
            elif key == ord('1'):
                calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEAN
                print('Switching calculation algorithm to MEAN!')
                newConfig = True
            elif key == ord('2'):
                calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MIN
                print('Switching calculation algorithm to MIN!')
                newConfig = True
            elif key == ord('3'):
                calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MAX
                print('Switching calculation algorithm to MAX!')
                newConfig = True
            elif key == ord('4'):
                calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MODE
                print('Switching calculation algorithm to MODE!')
                newConfig = True
            elif key == ord('5'):
                calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
                print('Switching calculation algorithm to MEDIAN!')
                newConfig = True

            if newConfig:
                config.roi = dai.Rect(topLeft, bottomRight)
                config.calculationAlgorithm = calculationAlgorithm
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(config)
                inputConfigQueue.send(cfg)
                newConfig = False