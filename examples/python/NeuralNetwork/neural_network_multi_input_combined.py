#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
from pathlib import Path

# Get the absolute path of the current script's directory
script_dir = Path(__file__).resolve().parent
examplesRoot = (script_dir / Path('../')).resolve()  # This resolves the parent directory correctly
models = examplesRoot / 'models'
tagImage = models / 'lenna.png'

# Decode the image using OpenCV
lenaImage = cv2.imread(str(tagImage.resolve()))
lenaImage = cv2.resize(lenaImage, (256, 256))
lenaImage = cv2.cvtColor(lenaImage, cv2.COLOR_BGR2RGB)
lenaImage = np.array(lenaImage)

device = dai.Device()
platform = device.getPlatform()
if platform == dai.Platform.RVC2:
    lenaImage = np.transpose(lenaImage, (2, 0, 1))
    nnTensorType = dai.TensorInfo.DataType.U8F
elif platform == dai.Platform.RVC4:
    # Add an empty dimension to the beginning
    lenaImage = np.expand_dims(lenaImage, axis=0)
    nnTensorType = dai.TensorInfo.DataType.FP16

inputNNData = dai.NNData()
inputNNData.addTensor("image1", lenaImage, dataType=nnTensorType)
inputNNData.addTensor("image2", lenaImage, dataType=nnTensorType)


with dai.Pipeline(device) as pipeline:
    model = dai.NNModelDescription("depthai-test-models/simple-concatenate-model")
    model.platform = platform.name

    nnArchive = dai.NNArchive(dai.getModelFromZoo(model))

    neuralNetwork = pipeline.create(dai.node.NeuralNetwork)
    neuralNetwork.setNNArchive(nnArchive)
    nnDataInputQueue = neuralNetwork.input.createInputQueue()
    qNNData = neuralNetwork.out.createOutputQueue()
    pipeline.start()
    while pipeline.isRunning():
        nnDataInputQueue.send(inputNNData)
        inNNData: dai.NNData = qNNData.get()
        tensor : np.ndarray = inNNData.getFirstTensor()
        # Drop the first dimension
        tensor = tensor.squeeze().astype(np.uint8)
        # Check the shape - in case 3 is not the last dimension, permute it to the last
        if tensor.shape[0] == 3:
            tensor = np.transpose(tensor, (1, 2, 0))
        cv2.imshow("Combined image", tensor)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break