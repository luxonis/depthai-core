#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import time

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()
    neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))

    qNNData = neuralNetwork.out.createOutputQueue()

    pipeline.start()


    while pipeline.isRunning():
        inNNData: dai.NNData = qNNData.get()
        tensor = inNNData.getFirstTensor()
        assert(isinstance(tensor, np.ndarray))
        print(f"Received NN data: {tensor.shape}")