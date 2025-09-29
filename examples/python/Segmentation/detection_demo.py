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
    
    model_description = dai.NNModelDescription("luxonis/yolov8-instance-segmentation-nano:coco-512x288", "RVC4")
    archive = dai.NNArchive(dai.getModelFromZoo(model_description))
    
    with dai.Pipeline(device) as pipeline:
        print("Creating pipeline...")

        input_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        
        cam_out = input_node.requestOutput((1920, 1080), dai.ImgFrame.Type.NV12, fps=fps)
        # cam_out = input_node.requestOutput((512, 288), dai.ImgFrame.Type.NV12, fps=fps)
        
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
        
        # sync_node = pipeline.create(dai.node.Sync)
        # stereo.depth.link(sync_node.inputs["depth"])
        # portNode.output.link(sync_node.inputs["img_detections"])
        
        # demux = pipeline.create(dai.node.MessageDemux)
        # sync_node.out.link(demux.input)
        
        # depth_overlay= pipeline.create(dai.node.Overlay)
        # depth_overlay.inputFrame.setBlocking(True)
        # depth_overlay.inputDetections.setBlocking(True)
        # demux.outputs["depth"].link(depth_overlay.inputFrame)
        # demux.outputs["img_detections"].link(depth_overlay.inputDetections)
        

        image_overlay = pipeline.create(dai.node.Overlay)
        image_overlay.inputDetections.setBlocking(True)
        image_overlay.inputFrame.setBlocking(True)
        cam_out.link(image_overlay.inputFrame)
        portNode.output.link(image_overlay.inputDetections)
        
        
        # crop_encoder = pipeline.create(dai.node.VideoEncoder)
        # crop_encoder.setMaxOutputFrameSize(1920 * 1088 * 3)
        # crop_encoder.setDefaultProfilePreset(
        #     fps, dai.VideoEncoderProperties.Profile.H264_MAIN
        # )
        # cam_out.link(crop_encoder.input)
        
        spatialLocationQueue = spatialLocationCalculator.out.createOutputQueue()
        spatialOutputQueue = spatialDetectionCalculator.out.createOutputQueue()

        # depth_overlay_out = depth_overlay.output.createOutputQueue()
        
        # cam_queue = crop_encoder.out.createOutputQueue()
        cam_queue = image_overlay.out.createOutputQueue()
        
        
        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            spatialData = spatialOutputQueue.get()
            # depth = stereoQueue.get()
            passthrough = cam_queue.get()
            # port_img_detections = portNodeQueue.get()
            spatialLocationsMsg = spatialLocationQueue.get()
            # depth_overlay_frame = depth_overlay_out.get()

            assert isinstance(spatialLocationsMsg, dai.SpatialLocationCalculatorData)
            # assert isinstance(port_img_detections, dai.ImgDetections)
            assert isinstance(spatialData, dai.SpatialImgDetections)
            assert isinstance(passthrough, dai.ImgFrame)
            # assert isinstance(depth_overlay_frame, dai.ImgFrame)
            # depth_overlay_image = depth_overlay_frame.getCvFrame()
            
            image = passthrough.getCvFrame()
            
            spatialLocations = spatialLocationsMsg.getSpatialLocations()
            
            # mask = port_img_detections.getCvSegmentationMask().astype(np.int32)
            # size = (512, 288)
            size = (1920, 1080)
            
            # mask = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
            
            # mask[mask == 255] = -1
            # scaled_mask = np.ones_like(mask, dtype=np.uint8) * 255
            # max_val = np.max(mask)
            # min_val = np.min(mask)
            
            # if min_val != max_val:
            #     scaled_mask = ((mask - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            
            # scaled_mask[mask == 255] = 0
            # scaled_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
            # scaled_mask[mask == -1] = image[mask == -1]
            
            # alpha = 0.7
            # image = cv2.addWeighted( image, alpha, scaled_mask, 1 - alpha, 0)
            
            depth_points = []
            
            for i, det in enumerate(spatialData.detections):
                outer_points = det.getBoundingBox()
                outer_points = outer_points.denormalize(image.shape[1], image.shape[0])
                outer_points = outer_points.getPoints()
                outer_points = [[int(p.x), int(p.y)] for p in outer_points]
                outer_points = np.array(outer_points, dtype=np.int32)
                cv2.polylines(image, [outer_points], isClosed=True, color=(0, 255, 0), thickness=2)
                
                # depth
                depth_coordinate = det.spatialCoordinates
                
                depth = det.depthMedian
                text = f"X: {int(depth_coordinate.x / 10 )}, Y: {int(depth_coordinate.y / 10)}, Z: {int(depth / 10)} cm"
                cv2.putText(image, text, outer_points[0], cv2.FONT_HERSHEY_PLAIN, 2, (232,36,87), 2)                
                
                # Location calculator
                if i < len(spatialLocations):
                    spatial_location = spatialLocations[i]
                    loc_coordinate = spatial_location.spatialCoordinates
                    text = f"LC X: {int(loc_coordinate.x / 10 )}, LC Y: {int(loc_coordinate.y / 10) }, LC Z: {int(loc_coordinate.z / 10)} cm"
                    cv2.putText(image, text, (outer_points[0][0], outer_points[0][1] + 35), cv2.FONT_HERSHEY_PLAIN, 2, (40, 40, 40), 2)
            
            
            # image = cv2.resize(image, (512*2, 288*2))
            # cv2.imshow("depth overlay", depth_overlay_image)
            # cv2.imshow("mask", scaled_mask)    
            cv2.imshow("depth", image)
            key = cv2.waitKey(1)
        
        
        