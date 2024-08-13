#!/usr/bin/env python3

import time
import depthai as dai

# We will download a blob NNArchive from the model zoo
# Pick your own model from
modelDescription = dai.NNModelDescription()
modelDescription.modelSlug = "yolov6-nano"
modelDescription.platform = "RVC2"

# Download model from zoo and load it
archivePath = dai.getModelFromZoo(modelDescription, useCached=True)
archive = dai.NNArchive(archivePath)

# Archive knows it is a blob archive
assert archive.getModelType() == dai.ModelType.SUPERBLOB

# Therefore, getSuperBlob() is available
assert archive.getSuperBlob() is not None

# The archive is unpacked and thus a path to the superblob model is also available
assert archive.getModelPath() is not None

# There is no blob available
assert archive.getBlob() is None

# One can access the NNArchive config as follows
config = archive.getConfig()

# You can access any config version
v1config: dai.nn_archive.v1.Config = config.getConfigV1()

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
    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(512, 288)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setInterleaved(False)

    # Neural network node
    blob = archive.getSuperBlob().getBlobWithNumShaves(6)
    neuralNetwork = pipeline.createNeuralNetwork()
    neuralNetwork.setBlob(blob)
    neuralNetwork.setNumInferenceThreads(2)

    # Linking
    camRgb.preview.link(neuralNetwork.input)

    nnDetectionQueue = neuralNetwork.out.createOutputQueue()
    nnPassthroughQueue = neuralNetwork.passthrough.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        in_nn = nnDetectionQueue.get()
        in_nnPassthrough = nnPassthroughQueue.get()

        print(in_nn)
        print(in_nnPassthrough)

        time.sleep(0.1)
