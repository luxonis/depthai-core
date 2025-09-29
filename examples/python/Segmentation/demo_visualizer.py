from pathlib import Path
import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from util_nodes import *

visualizer = dai.RemoteConnection(httpPort=8082)

print(dai.__file__)

if __name__ == "__main__":
    frame_type = dai.ImgFrame.Type.BGR888i
    fps =  30
    device = dai.Device()
    print("device type:", device.getPlatform())
    
    model_description = dai.NNModelDescription("luxonis/yolov8-instance-segmentation-nano:coco-512x288", "RVC4")
    archive = dai.NNArchive(dai.getModelFromZoo(model_description))
    
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        input_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        cam_out = input_node.requestOutput((1920, 1080), dai.ImgFrame.Type.BGR888i, fps=fps)
        
        image_manip = pipeline.create(dai.node.ImageManip)
        image_manip.setMaxOutputFrameSize(512*288*3)
        image_manip.initialConfig.setOutputSize(512, 288)
        image_manip.initialConfig.setFrameType(frame_type)
        cam_out.link(image_manip.inputImage)
        
        
        det_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            image_manip.out, "luxonis/yolov8-instance-segmentation-nano:coco-512x288",
        )
        
        portNode = pipeline.create(PortToDaiDetections)
        det_nn.out.link(portNode.input)
        
        
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
        
        spatialDetectionCalculator = pipeline.create(dai.node.SpatialDetectionCalculator)
        stereo.depth.link(spatialDetectionCalculator.inputDepth)
        portNode.output.link(spatialDetectionCalculator.inputDetections)
        
        config_creator = pipeline.create(CreateSpatialLocationCalculatorConfig)
        portNode.output.link(config_creator.input)
        
        
        spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
        spatialLocationCalculator.inputConfig.setWaitForMessage(True)
        stereo.depth.link(spatialLocationCalculator.inputDepth)
        config_creator.output.link(spatialLocationCalculator.inputConfig)
        
        overlay_node = pipeline.create(dai.node.Overlay)
        overlay_node.inputFrame.setBlocking(True)
        overlay_node.inputDetections.setBlocking(True)
        cam_out.link(overlay_node.inputFrame)
        portNode.output.link(overlay_node.inputDetections)

        
        annotation_node = pipeline.create(DepthAnnotations)
        spatialDetectionCalculator.out.link(annotation_node.inputSpatialDetections)
        spatialLocationCalculator.out.link(annotation_node.inputSpatialLocations)
        
        visualizer.addTopic("masked_frame", overlay_node.out)
        # visualizer.addTopic("color_frame", cam_out)
        visualizer.addTopic("detections", portNode.output)
        visualizer.addTopic("annotations", annotation_node.output)
        
        # visualizer.addTopic("masked_frame", cam_out)
        
        # spatialLocationQueue = spatialLocationCalculator.out.createOutputQueue()
        # spatialOutputQueue = spatialDetectionCalculator.out.createOutputQueue()
        # portNodeQueue = portNode.output.createOutputQueue()
        # # cam_queue = crop_encoder.out.createOutputQueue()
        # cam_queue = cam_out.createOutputQueue()
        
        
        print("Pipeline created.")

        pipeline.start()
        visualizer.registerPipeline(pipeline)

        while pipeline.isRunning():
            key = visualizer.waitKey(1)
            if key == ord("q"):
                print("Got q key. Exiting...")
                break
        
        