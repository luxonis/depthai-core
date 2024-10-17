#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import requests
import time

# API of the Hub team with test DAI models
API_KEY = "tapi.RKIViXv_2poidTBST5CSAQ.fdj2dDGH6gADEIsowwAvjFIvgY2fa2Rj5bmhADXrdXpPOBS_eGdHGM2GcaEhhESFlYip_9mFb0BRqQ-fP3gy1A"


# Get the Lenna image
url = "https://upload.wikimedia.org/wikipedia/en/7/7d/Lenna_%28test_image%29.png"

# Download the image from the web
response = requests.get(url)
image_array = np.asarray(bytearray(response.content), dtype=np.uint8)

# Decode the image using OpenCV
lenaImage = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
lenaImage = cv2.resize(lenaImage, (256, 256))

device = dai.Device()
platform = device.getPlatform()
if(platform == dai.Platform.RVC2):
    daiType = dai.ImgFrame.Type.RGB888p
elif(platform == dai.Platform.RVC4):
    daiType = dai.ImgFrame.Type.RGB888i
else:
    raise RuntimeError("Platform not supported")

daiLenaImage = dai.ImgFrame()

daiLenaImage.setCvFrame(lenaImage, daiType)

with dai.Pipeline(device) as pipeline:
    model = dai.NNModelDescription("simple-concatenate-model")
    model.platform = platform.name

    nnArchive = dai.NNArchive(dai.getModelFromZoo(model, apiKey=API_KEY))
    cam = pipeline.create(dai.node.Camera).build()
    camOut = cam.requestOutput((256,256), daiType)

    neuralNetwork = pipeline.create(dai.node.NeuralNetwork)
    neuralNetwork.setNNArchive(nnArchive)
    camOut.link(neuralNetwork.inputs["image1"])
    lennaInputQueue = neuralNetwork.inputs["image2"].createInputQueue()
    # No need to send the second image everytime
    neuralNetwork.inputs["image2"].setReusePreviousMessage(True)
    qNNData = neuralNetwork.out.createOutputQueue()
    pipeline.start()
    lennaInputQueue.send(daiLenaImage)
    while pipeline.isRunning():
        inNNData: dai.NNData = qNNData.get()
        tensor : np.ndarray = inNNData.getFirstTensor()
        # Drop the first dimension
        tensor = tensor.squeeze().astype(np.uint8)
        # Check the shape - in case 3 is not the last dimension, permute it to the last
        if tensor.shape[0] == 3:
            tensor = np.transpose(tensor, (1, 2, 0))
        print(tensor.shape)
        cv2.imshow("Combined", tensor)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break