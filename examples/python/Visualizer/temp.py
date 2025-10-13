#!/usr/bin/env python3
import depthai as dai
import cv2
import numpy as np

model_name = "luxonis/yolov6-nano:r2-coco-512x288"
model_name = "luxonis/ppe-detection:640x640"
nn_archive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription(model_name, platform="RVC4")))

# remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    
    nn = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, dai.NNModelDescription(model_name))
    # nn = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))
    
    detectionNetwork = pipeline.create(dai.node.DetectionParser).build(nn.out, nn_archive)

    cam_queue = nn.passthrough.createOutputQueue()
    detectionQueue = detectionNetwork.out.createOutputQueue()
    # cam_queue = cameraNode.requestOutput((640, 640), dai.ImgFrame.Type.BGR888i, fps=30).createOutputQueue()
    
    pipeline.start()
    print("Pipeline started")
    # remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        passthrough = cam_queue.get()
        detections = detectionQueue.get()
        assert isinstance(detections, dai.ImgDetections)
        assert isinstance(passthrough, dai.ImgFrame)
        image= passthrough.getCvFrame()
        
        width = image.shape[1]
        height = image.shape[0]
        
        for i, det in enumerate(detections.detections):
            xmin, ymin, xmax, ymax = int(det.xmin * width), int(det.ymin * height), int(det.xmax * width), int(det.ymax * height)
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
            conf = det.confidence
            label = det.label
            cv2.putText(image, f"{label} - {int(conf*100)}%", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            
            
        cv2.imshow("DAI NODES IMPL", image)
        if cv2.waitKey(1) == ord('q'):
            break            
            
        
            
