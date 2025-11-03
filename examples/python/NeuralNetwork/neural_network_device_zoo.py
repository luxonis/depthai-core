#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import time

# Create pipeline
with dai.Pipeline() as pipeline:
    cameraNode = pipeline.create(dai.node.Camera).build()
    # Longer form - useful in case of a local NNArchive
    # modelDescription = dai.NNModelDescription("yolov6-nano", platform=pipeline.getDefaultDevice().getPlatformAsString())
    # archive = dai.NNArchive(dai.getModelFromZoo(modelDescription))
    # neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, archive)
    neuralNetwork = pipeline.create(dai.node.NeuralNetwork)
    left = cameraNode.requestOutput((768, 480), fps=10, type=dai.ImgFrame.Type.GRAY8)
    right = cameraNode.requestOutput((768, 480), fps=10, type=dai.ImgFrame.Type.GRAY8)
    left.link(neuralNetwork.inputs["left"])
    right.link(neuralNetwork.inputs["right"])
    neuralNetwork.setModelFromDeviceZoo(dai.node.NeuralNetwork.DeviceModelZoo.NEURAL_DEPTH_LARGE)
    qNNData = neuralNetwork.out.createOutputQueue()

    pipeline.start()


    while pipeline.isRunning():
        inNNData: dai.NNData = qNNData.get()
        tensor = inNNData.getFirstTensor()
        assert(isinstance(tensor, np.ndarray))
        print(f"Received NN data: {tensor.shape}")
