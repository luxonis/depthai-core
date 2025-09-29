import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionExtended, ImgDetectionsExtended, Keypoint
import cv2

from util_nodes import *

print(dai.__file__)

if __name__ == "__main__":
    frame_type = dai.ImgFrame.Type.BGR888i
    fps =  30
    device = dai.Device()
    print("device type:", device.getPlatform())
    
    
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        
        monoLeftOut = monoLeft.requestOutput((512, 288))
        monoRightOut = monoRight.requestOutput((512, 288))
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)
        
        stereo.setRectification(True)
        stereo.setExtendedDisparity(True)

        stepSize = 0.05
        
        stereoQueue = stereo.depth.createOutputQueue()        
        
        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            depth_overlay_frame = stereoQueue.get()

            assert isinstance(depth_overlay_frame, dai.ImgFrame)
            
            depth_overlay_image = depth_overlay_frame.getCvFrame()            

            cv2.imshow("depth overlay", depth_overlay_image)
            key = cv2.waitKey(1)
        
        
        