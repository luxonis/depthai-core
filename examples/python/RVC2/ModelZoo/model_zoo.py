#!/usr/bin/env python3

import time
import depthai as dai

# Describe the model I want to download - two options: in program or from yaml file

# Option 1: In program
# modelDescription = dai.NNModelDescription()
# modelDescription.modelSlug = "yolov6-nano"
# modelDescription.platform = "RVC2"

# Option 2: From yaml file
modelDescription = dai.NNModelDescription.fromYamlFile("mymodel.yaml")

# If you want, you can store the model description in a yaml file
modelDescription.saveToYamlFile("mymodel.yaml")

# Return path to downloaded model - yolov6-nano-r2-288x512.tar.xz for this example
modelPath = dai.getModelFromZoo(modelDescription, useCached=True)
print("Using model cached at path: ", modelPath)


archive = dai.NNArchive(modelPath)
blob = archive.getSuperBlob().getBlobWithNumShaves(6)
print("Number of shaves: ", blob.numShaves)
print(archive.getConfig().getConfigV1().model.heads[0].metadata.classes)

# Construct pipeline and start using downloaded NN model :-)
with dai.Pipeline() as pipeline:

    # It's even possible to download a model for the used device
    # modelDescription = dai.NNModelDescription()
    # modelDescription.modelSlug = "yolov6-nano"

    #  The two are equivalent
    # modelDescription.platform = pipeline.getDefaultDevice().getPlatformAsString()
    # modelDescription = dai.platform2string(pipeline.getDefaultDevice().getPlatform())

    # Color camera node
    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(512, 288)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setInterleaved(False)

    # Neural network node
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

        # Do whatever is needed with the inference results
        print(in_nn)
        print(in_nnPassthrough)

        time.sleep(0.1)
