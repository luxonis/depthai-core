#!/usr/bin/env python3

import time
import depthai as dai

# We will download a blob NNArchive from the model zoo
# Pick your own model from
modelDescription = dai.NNModelDescription()
modelDescription.model = "yolov6-nano"
modelDescription.platform = "RVC2"

# Download model from zoo and load it
archivePath = dai.getModelFromZoo(modelDescription, useCached=True)
archive = dai.NNArchive(archivePath)

# Archive knows it is a blob archive
assert archive.getModelType() == dai.ModelType.SUPERBLOB

# Therefore, getSuperBlob() is available
assert archive.getSuperBlob() is not None

# There is no blob or other model format available
assert archive.getBlob() is None
assert archive.getOtherModelFormat() is None

# You can access any config version
v1config: dai.nn_archive.v1.Config = archive.getConfig()

# Print some config fields
print("-" * 10)
print("Config fields:")
print(f"\tConfig version: {v1config.configVersion}")
print(f"\tModel heads: {v1config.model.heads}")
print(f"\tModel inputs: {v1config.model.inputs}")
print(f"\tModel metadata: {v1config.model.metadata}")
print(f"\tModel outputs: {v1config.model.outputs}")
print("-" * 10)

with dai.Pipeline() as pipeline:
    # Color camera node
    camRgb = pipeline.create(dai.node.Camera).build()
    outCam = camRgb.requestOutput((416, 416), dai.ImgFrame.Type.BGR888p)

    # Neural network node
    blob = archive.getSuperBlob().getBlobWithNumShaves(6)
    neuralNetwork = pipeline.create(dai.node.NeuralNetwork)
    neuralNetwork.setBlob(blob)
    neuralNetwork.setNumInferenceThreads(2)

    # Linking
    outCam.link(neuralNetwork.input)

    nnDetectionQueue = neuralNetwork.out.createOutputQueue()
    nnPassthroughQueue = neuralNetwork.passthrough.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        in_nn = nnDetectionQueue.get()
        in_nnPassthrough = nnPassthroughQueue.get()
        print("Data received")
        time.sleep(0.1)
