import time
import depthai as dai
import numpy as np
from depthai_nodes.message import ImgDetectionExtended, ImgDetectionsExtended, Keypoint
import cv2


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
            dets = []
            remove_ids = []
            for i, det in enumerate(in_data.detections):
                if det.label_name != "person":
                    remove_ids.append(i)
                    continue
                dai_det = dai.ImgDetection()
                dai_det.label = det.label
                dai_det.confidence = det.confidence
                dai_det.setBoundingBox(det.rotated_rect)
                keypoints = []
                for kp in det.keypoints:
                    keypoints.append( dai.Keypoint(kp.x, kp.y, kp.z))
                dai_det.setKeypoints(keypoints)
                
                dets.append(dai_det)
            
            dai_detections.detections = dets
            dai_detections.setTimestamp(in_data.getTimestamp())
            dai_detections.setSequenceNum(in_data.getSequenceNum())
            dai_detections.setTransformation(in_data.getTransformation())
            dai_detections.setTimestampDevice(in_data.getTimestampDevice())
            
            mask = in_data.masks
            mask[mask == -1 ] = 255
            for rid in remove_ids:
                mask[mask == rid] = 255
            
            unique_ids = np.unique(mask)
            id_map = {old_id: new_id for new_id, old_id in enumerate(unique_ids) if old_id != 255}
            for old_id, new_id in id_map.items():
                mask[mask == old_id] = new_id
            
            
            mask = mask.astype(np.uint8)
            dai_detections.setSegmentationMask(mask)

            self.output.send(dai_detections)
            
            
class CreateSpatialLocationCalculatorConfig(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.output = self.createOutput()
        
    def run(self) -> None:
        while True:
            in_data = self.input.get()
            if in_data is None:
                continue
            assert isinstance(in_data, dai.ImgDetections)
            
            configs = dai.SpatialLocationCalculatorConfig()
            for det in in_data.detections:
                config = dai.SpatialLocationCalculatorConfigData()
                rotated_rect: dai.RotatedRect = det.getBoundingBox()
                roi: dai.Rect = dai.Rect(*rotated_rect.getOuterXYWH())
                
                config.depthThresholds.lowerThreshold = 0
                config.depthThresholds.upperThreshold = 65535
                config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
                config.roi = roi
                
                configs.addROI(config)
                
            self.output.send(configs)
            
            
class OverlayFrame(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.inputFrame = self.createInput("frame")
        self.inputDetections = self.createInput("detections")
        self.output = self.createOutput("masked_frame")
        
    
    def run(self) -> None:
        while True:
            detections = self.inputDetections.get()
            frame = self.inputFrame.get()
            
            assert isinstance(frame, dai.ImgFrame)
            assert isinstance(detections, dai.ImgDetections)
            
            image = frame.getCvFrame()
            mask = detections.getSegmentationMask().astype(np.int32)
            
            size = (512, 288)
            size = (1920, 1080)
            
            mask = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
            
            if frame.getWidth() != size[0] or frame.getHeight() != size[1]:
                image = cv2.resize(image, size, interpolation=cv2.INTER_LINEAR)
            
            if len(image.shape) == 2:
                image = image / np.max(image) * 255
                image = image.astype(np.uint8)
                image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
                
                
                
            mask[mask == 255] = -1
            scaled_mask = np.ones_like(mask, dtype=np.uint8) * 255
            max_val = np.max(mask)
            min_val = np.min(mask)
            
            if min_val != max_val:
                scaled_mask = ((mask - min_val) / (max_val - min_val) * 255).astype(np.uint8)
            
            scaled_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
            
            start = time.time() # THIS IS THE PROBLEM
            scaled_mask[mask == -1] = image[mask == -1]
            
            print(f"OverlayFrame processing time: {time.time() - start:.03f}s")
            # image[mask != -1] = scaled_mask[mask != -1] * 0.5 + image[mask != -1] * 0.5 
            
            alpha = 0.2
            image = cv2.addWeighted( image, alpha, scaled_mask, 1 - alpha, 0)
            
            masked_frame = dai.ImgFrame()
            masked_frame.setFrame(image)
            masked_frame.setType(dai.ImgFrame.Type.BGR888i)
            masked_frame.setWidth(image.shape[1])
            masked_frame.setHeight(image.shape[0])
            masked_frame.setTimestamp(frame.getTimestamp())
            masked_frame.setSequenceNum(frame.getSequenceNum())
            masked_frame.setTimestampDevice(frame.getTimestampDevice())
            masked_frame.setTransformation(frame.getTransformation())
            
            self.output.send(masked_frame)

class SyncDecoder(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.out_frame = self.createOutput()
        self.out_detections = self.createOutput()
    
    def run(self) -> None:
        while True:
            in_data = self.input.get()
            if in_data is None:
                continue
            
            out_frame = in_data["depth"]
            out_detections = in_data["img_detections"]
            self.out_frame.send(out_frame)
            self.out_detections.send(out_detections)
            
class DepthAnnotations(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.inputSpatialDetections = self.createInput()
        self.inputSpatialLocations = self.createInput()
        self.output = self.createOutput()
        
    def run(self) -> None:
        while True:
            detections = self.inputSpatialDetections.get()
            spatialLocationsMsg = self.inputSpatialLocations.get()
            
            assert isinstance(spatialLocationsMsg, dai.SpatialLocationCalculatorData)
            assert isinstance(detections, dai.SpatialImgDetections)
            imgAnnt = dai.ImgAnnotations()
            imgAnnt.setTimestamp(detections.getTimestamp())
            spatialLocations = spatialLocationsMsg.getSpatialLocations()
            
            annotation = dai.ImgAnnotation()
            backgroundColor = dai.Color(1.0, 1.0, 1.0, 0.7)
            for i, detection in enumerate(detections.detections):
                
                text_position = dai.Point2f(detection.xmin, detection.ymin)
                
                
                text = dai.TextAnnotation()
                text.position = text_position
                depth_coordinate = detection.spatialCoordinates
                
                text.text = f"X: {int(depth_coordinate.x / 10)}cm Y: {int(depth_coordinate.y / 10)}cm Z: {int(depth_coordinate.z / 10)}cm"
                text.fontSize = 40
                text.textColor = dai.Color( float(88 / 255), float(21 / 255), float(127 / 255), float(1.0))
                text.backgroundColor = backgroundColor
                
                annotation.texts.append(text)
                
                if i < len(spatialLocations):
                    location_text = dai.TextAnnotation()
                    location_text.position = dai.Point2f(detection.xmin, detection.ymin + 0.05)
                    spatial_location = spatialLocations[i]
                    loc_coordinate = spatial_location.spatialCoordinates
                    location_text.text = f"LC X: {int(loc_coordinate.x / 10)}cm Y: {int(loc_coordinate.y / 10)}cm Z: {int(loc_coordinate.z / 10)}cm"
                    location_text.fontSize = 40
                    location_text.textColor = dai.Color( float(25 / 255), float(25 / 255), float(25 / 255), float(1.0))
                    location_text.backgroundColor = backgroundColor
                    
                    annotation.texts.append(location_text)
                    
                    

            imgAnnt.annotations.append(annotation)
            self.output.send(imgAnnt)
            
            
class PortToDaiDetections2(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput("in")
        self.output = self.createOutput("out")

        
    def run(self) -> None:
        while True:
            in_data = self.input.get()
            
            
            dai_detections = dai.ImgDetections()
            dets = []
            dai_det = dai.ImgDetection()
            
            dai_det.label = 1
            dai_det.confidence = 0.7
            dets.append(dai_det)
        
            dai_detections.detections = dets

            mask = np.ones((288, 512), dtype=np.uint8) * 255
            mask[100:200, 100:200] = 0            
            
            dai_detections.setSegmentationMask(mask)

            self.output.send(dai_detections)