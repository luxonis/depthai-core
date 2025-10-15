#!/usr/bin/env python3
import depthai as dai
import cv2
import numpy as np
from depthai_nodes.node import YOLOExtendedParser, ParsingNeuralNetwork
from depthai_nodes.message import ImgDetectionsExtended


# remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build(sensorFps=30.0)
    
    cam_queue = cameraNode.requestOutput((640, 480), dai.ImgFrame.Type.BGR888i, fps=30.0).createOutputQueue()
    
    pipeline.start()
    print("Pipeline started")
    # remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        passthrough = cam_queue.get()
        # detections = detectionQueue.get()
        # dai_nodes_results = dai_nodes_nn_out.get()
        print("----")
        
        # assert isinstance(dai_nodes_results, (ImgDetectionsExtended, dai.ImgDetections))
        # assert isinstance(detections, dai.ImgDetections)
        assert isinstance(passthrough, dai.ImgFrame)
        image = passthrough.getCvFrame()
        
        
        cv2.imshow("DAI NODES IMPL", image)
        if cv2.waitKey(1) == ord('q'):
            break            
            
            
