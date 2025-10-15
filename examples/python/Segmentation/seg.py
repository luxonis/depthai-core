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
        
        
        det_nn2 = pipeline.create(dai.node.NeuralNetwork).build(image_manip.out, archive )
        det_parser = pipeline.create(dai.node.YoloSegmentationParser)
        det_nn2.out.link(det_parser.input)
        
        
        stepSize = 0.05
        
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
        det_parser.out.link(image_overlay.inputDetections)
        
        
        # crop_encoder = pipeline.create(dai.node.VideoEncoder)
        # crop_encoder.setMaxOutputFrameSize(1920 * 1088 * 3)
        # crop_encoder.setDefaultProfilePreset(
        #     fps, dai.VideoEncoderProperties.Profile.H264_MAIN
        # )
        # cam_out.link(crop_encoder.input)
        

        # depth_overlay_out = depth_overlay.output.createOutputQueue()
        
        # cam_queue = crop_encoder.out.createOutputQueue()
        cam_queue = image_overlay.out.createOutputQueue()
        
        
        print("Pipeline created.")

        pipeline.start()

        while pipeline.isRunning():
            # depth = stereoQueue.get()
            passthrough = cam_queue.get()
            # port_img_detections = portNodeQueue.get()
            # depth_overlay_frame = depth_overlay_out.get()

            assert isinstance(passthrough, dai.ImgFrame)
            # assert isinstance(depth_overlay_frame, dai.ImgFrame)
            # depth_overlay_image = depth_overlay_frame.getCvFrame()
            
            image = passthrough.getCvFrame()
            
            
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
            
            # image = cv2.resize(image, (512*2, 288*2))
            # cv2.imshow("depth overlay", depth_overlay_image)
            # cv2.imshow("mask", scaled_mask)    
            cv2.imshow("segmentation", image)
            key = cv2.waitKey(1)
        
        
        