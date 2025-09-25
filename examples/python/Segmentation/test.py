import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionExtended, ImgDetectionsExtended, Keypoint
import cv2

print(dai.__file__)
class PortToDaiDetections(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput("in")
        self.output = self.createOutput("out")

        
    def run(self) -> None:
        while True:
            in_data = self.input.get()
            
            if in_data is None:
                continue
            
            assert isinstance(in_data, ImgDetectionsExtended)
            
            dai_detections = dai.ImgDetections()
            for det in in_data.detections:
                dai_det = dai.ImgDetection()
                dai_det.label = det.label
                dai_det.confidence = det.confidence
                dai_det.setBoundingBox(det.rotated_rect)
                keypoints = []
                for kp in det.keypoints:
                    keypoints.append( dai.Keypoint(kp.x, kp.y, kp.z))
                dai_det.setKeypoints(keypoints)
                
                dai_detections.detections.append(dai_det)
            
            dai_detections.setTimestamp(in_data.getTimestamp())
            dai_detections.setSequenceNum(in_data.getSequenceNum())
            dai_detections.setTransformation(in_data.getTransformation())
            dai_detections.setTimestampDevice(in_data.getTimestampDevice())
            
            mask = np.ones((640, 480), dtype=np.uint8) * 255
            mask[0:320, 0:480] = 0
            mask[320:480, 240:480] = 1

            dai_detections.setSegmentationMask(mask)
            
            self.output.send(dai_detections)

if __name__ == "__main__":
    frame_type = dai.ImgFrame.Type.BGR888i
    fps =  30
    device = dai.Device()
    print("device type:", device.getPlatform())
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        input_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

        det_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            input_node, "luxonis/yunet:640x480", fps=fps
        )
        
        portNode = pipeline.create(PortToDaiDetections)
        det_nn.out.link(portNode.input)
        
        
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        
        monoLeftOut = monoLeft.requestOutput((640, 480))
        monoRightOut = monoRight.requestOutput((640, 480))
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)
        
        stereo.setRectification(True)
        stereo.setExtendedDisparity(True)

        stepSize = 0.05
        
        spatialDetectionCalculator = pipeline.create(dai.node.SpatialDetectionCalculator)
        stereo.depth.link(spatialDetectionCalculator.inputDepth)
        portNode.output.link(spatialDetectionCalculator.inputDetections)
        
        outputDepthQueue = spatialDetectionCalculator.passthroughDepth.createOutputQueue()
        spatialOutputQueue = spatialDetectionCalculator.out.createOutputQueue()
        # stereoQueue = stereo.depth.createOutputQueue()
        passthroughQueue = det_nn.passthrough.createOutputQueue()
        

        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            spatialData = spatialOutputQueue.get()
            passthrough = passthroughQueue.get()
            # depth = stereoQueue.get()
            
            assert isinstance(passthrough, dai.ImgFrame)
            assert isinstance(spatialData, dai.SpatialImgDetections)
            # assert isinstance(depth, dai.ImgFrame)
            image = passthrough.getCvFrame()
            # image = passthrough.getFrame()
            # depthFrame = depth.getFrame()
            
            for det in spatialData.detections:
                outer_points = det.getBoundingBox().getOuterPoints()
                outer_points = np.array(outer_points, dtype=np.int32)
                cv2.polylines(image, [outer_points], isClosed=True, color=(0, 255, 0), thickness=2)
                
            
            
            # cv2.normalize(depthFrame, depthFrame, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            # depthFrameColor = cv2.equalizeHist(depthFrame)
            # depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
            
            cv2.imshow("depth", image)
            key = cv2.waitKey(1)
        
        
        