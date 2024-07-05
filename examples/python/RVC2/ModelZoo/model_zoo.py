#!/usr/bin/env python3

import time
import depthai as dai

# Describe the model I want to download - two options: in program or from yaml file

# Option 1: In program
# modelDescription = dai.NNModelDescription()
# modelDescription.modelSlug = "ales-test"
# modelDescription.modelPath = "RVC2"

# Option 2: From yaml file
modelDescription = dai.NNModelDescription.fromYamlFile("mymodel.yaml")

# Return path to downloaded model - resnet18_xxx.blob for this example
modelPath = dai.getModelFromZoo(modelDescription, verbose=True, useCached=True)

print("Using model cached at path: ", modelPath)

assert modelPath.endswith(".blob") # We expect a blob for this particular example

# Construct pipeline and start using downloaded NN model :-)
with dai.Pipeline() as pipeline:

    # It's even possible to download a model for the used device
    # modelDescription = dai.NNModelDescription()
    # modelDescription.modelSlug = "ales-test"

    #  The two are equivalent
    # modelDescription.platform = pipeline.getDefaultDevice().getPlatformAsString()
    # modelDescription = dai.platform2string(pipeline.getDefaultDevice().getPlatform())

    # Color camera node
    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(256, 256)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setInterleaved(False)

    # Neural network node
    neuralNetwork = pipeline.createNeuralNetwork()
    neuralNetwork.setBlobPath(modelPath)
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