import time
import depthai as dai

import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionExtended, ImgDetectionsExtended, Keypoint
import cv2


                                

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
        

        det_nn = pipeline.create(dai.node.DetectionNetwork).build(image_manip.out, archive, 0.5)
        
        
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
        
        
        spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
        spatialLocationCalculator.inputConfig.setWaitForMessage(True)
        stereo.depth.link(spatialLocationCalculator.inputDepth)
        config_creator.output.link(spatialLocationCalculator.inputConfig)
        
        
        # crop_encoder = pipeline.create(dai.node.VideoEncoder)
        # crop_encoder.setMaxOutputFrameSize(1920 * 1088 * 3)
        # crop_encoder.setDefaultProfilePreset(
        #     fps, dai.VideoEncoderProperties.Profile.H264_MAIN
        # )
        # cam_out.link(crop_encoder.input)
        
        spatialLocationQueue = spatialLocationCalculator.out.createOutputQueue()
        spatialOutputQueue = spatialDetectionCalculator.out.createOutputQueue()
        portNodeQueue = portNode.output.createOutputQueue()
        # cam_queue = crop_encoder.out.createOutputQueue()
        cam_queue = cam_out.createOutputQueue()
        
        
        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            spatialData = spatialOutputQueue.get()
            # depth = stereoQueue.get()
            passthrough = cam_queue.get()
            port_img_detections = portNodeQueue.get()
            spatialLocationsMsg = spatialLocationQueue.get()

            assert isinstance(spatialLocationsMsg, dai.SpatialLocationCalculatorData)
            assert isinstance(port_img_detections, dai.ImgDetections)
            assert isinstance(spatialData, dai.SpatialImgDetections)
            assert isinstance(passthrough, dai.ImgFrame)
            image = passthrough.getCvFrame()
            
            spatialLocations = spatialLocationsMsg.getSpatialLocations()
            
            mask = port_img_detections.getCvSegmentationMask().astype(np.int32)
            mask = cv2.resize(mask, (1920, 1080), interpolation=cv2.INTER_NEAREST)
            
            mask[mask == 255] = -1
            scaled_mask = np.ones_like(mask, dtype=np.uint8) * 255
            max_val = np.max(mask)
            min_val = np.min(mask)
            
            if min_val != max_val:
                scaled_mask = ((mask - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            
            scaled_mask[mask == 255] = 0
            scaled_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
            scaled_mask[mask == -1] = image[mask == -1]
            
            alpha = 0.7
            image = cv2.addWeighted( image, alpha, scaled_mask, 1 - alpha, 0)
            
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
            cv2.imshow("mask", scaled_mask)    
            cv2.imshow("depth", image)
            key = cv2.waitKey(1)
        
        
        