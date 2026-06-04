import time
from typing import Optional, Tuple

import cv2
import depthai as dai

class CropConfigsCreator(dai.node.ThreadedHostNode):

    def __init__(self) -> None:
        """Initializes the node."""
        super().__init__()
        self.config_output = self.createOutput()
        self.detections_input = self.createInput()

    def run(self) -> None:
        while (self.isRunning()):
            detections_input = self.detections_input.get()
            assert isinstance(detections_input, dai.ImgDetections)
            
            detections = detections_input.detections
            
            cfg = dai.ImageManipConfig()
            cfg.setReusePreviousImage(False)
            for i in range(len(detections)):
                detection: dai.ImgDetection = detections[i]
                
                # print corners of the bounding box
                bbox = detection.getBoundingBox()
                outer_bbox = bbox.getOuterRect()
                x1 = max(0.0, min(outer_bbox[0], 0.9999))
                y1 = max(0.0, min(outer_bbox[1], 0.9999))
                x2 = max(0.0, min(outer_bbox[2], 0.9999))
                y2 = max(0.0, min(outer_bbox[3], 0.9999))
                bbox = dai.RotatedRect(dai.Rect(x1, y1, x2 - x1, y2 - y1, True))
                
                # print(f"Detection {i}: bbox corners:")
                # corners = bbox.getPoints()
                # for j in range(4):
                #     corner = corners[j]
                #     print(f"  Corner {j}: ({corner.x:.4f}, {corner.y:.4f})")

                cfg.addCropRotatedRect(bbox, normalizedCoords=True)
                cfg.setOutputSize(200, 200, mode=dai.ImageManipConfig.ResizeMode.STRETCH)
                if(i == 0):
                    cfg.setReusePreviousImage(False)
                else:
                    cfg.setReusePreviousImage(True)

                send_status = False
                attempts = 0
                while (
                    not send_status and attempts < 100):
                    send_status = self.config_output.trySend(cfg)
                    if not send_status:
                        attempts += 1
                        time.sleep(0.001)

if __name__ == "__main__":
    # Create pipeline
    pipeline = dai.Pipeline()

    # Create nodes
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    
    
    detNN = pipeline.create(dai.node.DetectionNetwork).build(cam, dai.NNModelDescription("luxonis/yolov8-instance-segmentation-large:coco-640x480"))
    
    full_res_output = cam.requestOutput((1280, 800))
    
    crop_configs_creator = pipeline.create(CropConfigsCreator)
    detNN.out.link(crop_configs_creator.detections_input)
    
    
    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.inputConfig.setReusePreviousMessage(False)
    
    imageManip.setMaxOutputFrameSize(300 * 480 * 3)
    crop_configs_creator.config_output.link(imageManip.inputConfig)
    full_res_output.link(imageManip.inputImage)
    
    cropQ = imageManip.out.createOutputQueue()
    detQ = detNN.out.createOutputQueue()
    passthroughQ = detNN.passthrough.createOutputQueue()
    
    # Start pipeline
    pipeline.start()
    
    color = (255, 0, 0)
    while(pipeline.isRunning()):
        crops = cropQ.get()
        detections = detQ.get()
        passthrough = passthroughQ.get()
        assert isinstance(crops, dai.ImgFrame)
        assert isinstance(detections, dai.ImgDetections)
        assert isinstance(passthrough, dai.ImgFrame)
        
        print(f"Got {len(detections.detections)} detections)")
        
        frame = passthrough.getCvFrame()
        height, width = frame.shape[:2]
        for detection in detections.detections:
            x1 = int(detection.xmin * width)
            y1 = int(detection.ymin * height)
            x2 = int(detection.xmax * width)
            y2 = int(detection.ymax * height)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        cv2.imshow("passthrough", frame)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break

    cv2.destroyAllWindows()
