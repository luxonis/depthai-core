#!/usr/bin/env python3
import depthai as dai
import cv2
import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionsExtended

# model_name = "luxonis/yolov6-nano:r2-coco-512x288"
# model_name = "luxonis/yolov6-nano:r2-coco-512x384"
# model_name = "luxonis/yolov8-instance-segmentation-nano:coco-512x288"
# model_name = "luxonis/yolov8-large-pose-estimation:coco-640x352"
# model_name = "luxonis/yolo-p:bdd100k-320x320"
# model_name = "luxonis/yolov8-instance-segmentation-large:coco-640x352"
# model_name = "agmo/yolov11n-512x288:model-variant-1"
# model_name = "luxonis/yolov10-nano:coco-512x288"
# model_name = "luxonis/barcode-detection:512x384"
model_name = "luxonis/ppe-detection:640x640"
# model_name = "luxonis/yoloe-v8-l:640x640"
archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription(model_name, platform="RVC4")))

# remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    nn = pipeline.create(ParsingNeuralNetwork).build(cameraNode, dai.NNModelDescription(model_name), 30)
    
    # nn = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, dai.NNModelDescription(model_name))
    
    # detectionNetwork = pipeline.create(dai.node.DetectionParser).build(nn.out, archive )
    
    # remoteConnector.addTopic("detections", detectionNetwork.out, "img")
    # remoteConnector.addTopic("images", nn.passthrough, "img")
    cam_queue = nn.passthrough.createOutputQueue()
    detectionQueue = nn.getOutput(0).createOutputQueue()
    
    pipeline.start()
    # remoteConnector.registerPipeline(pipeline)
    print("Starting pipeline...")
    while pipeline.isRunning():
        passthrough = cam_queue.get()
        detections = detectionQueue.get()
        
        assert isinstance(detections, ImgDetectionsExtended)
        assert isinstance(passthrough, dai.ImgFrame)
        image = passthrough.getCvFrame()
        height, width = image.shape[:2]
        for i, det in enumerate(detections.detections):
            outer_points = det.rotated_rect
            outer_points = outer_points.denormalize(image.shape[1], image.shape[0])
            outer_points = outer_points.getPoints()
            outer_points = [[int(p.x), int(p.y)] for p in outer_points]
            outer_points = np.array(outer_points, dtype=np.int32)
            cv2.polylines(image, [outer_points], isClosed=True, color=(0, 255, 0), thickness=2)
            conf = det.confidence
            cv2.putText(image, f"{int(conf*100)}%", (outer_points[0][0], outer_points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            keypoints = det.keypoints
            for kp in keypoints:
                print((kp.x, kp.y))
                cv2.circle(image, (int(kp.x * width), int(kp.y * height)), 3, (0, 0, 255), -1)
            
        
        blended = image
        # segmentation_mask = detections.getCvSegmentationMask()
        # if segmentation_mask is not None:
            
        #     mat = cv2.Mat(segmentation_mask)
        #     mask = mat == 255
        #     cv2.normalize(mat, mat, 0, 254, cv2.NORM_MINMAX, cv2.CV_8U)
        #     # make all values equaldistant between 0-255
        #     # e.g. if there are 4 unique values, make them 0, 85, 170, 255
            
        #     num_unique = len(np.unique(mat))
            
        #     if num_unique > 1:
        #         distance_step = 255 // (num_unique)
        #         mat[~mask] = (mat[~mask] * distance_step).astype(np.uint8) 
            
                        
        #     mat = cv2.applyColorMap(mat, cv2.COLORMAP_JET)
        #     mat[mask] = image[mask]
        #     blended = cv2.addWeighted(image, 0.7, mat, 0.3, 0)
            
        cv2.imshow("detections", blended)
        if cv2.waitKey(1) == ord('q'):
            break            
            
            
