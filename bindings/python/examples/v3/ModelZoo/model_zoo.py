#!/usr/bin/env python3

import time
import depthai as dai

# Describe the model I want to download - two options: in program or from yaml file

# Option 1: In program
# model_description = dai.NNModelDescription()
# model_description.modelSlug = "ales-test"
# model_description.modelPath = "RVC2"

# Option 2: From yaml file
model_description = dai.NNModelDescription.fromYamlFile("mymodel.yaml")

# Return path to downloaded model - resnet18_xxx.blob for this example
model_path = dai.getModelFromZoo(model_description, verbose=True, useCached=True)

print("Using model cached at path: ", model_path)

assert model_path.endswith(".blob") # We expect a blob for this particular example

# Construct pipeline and start using downloaded NN model :-)
with dai.Pipeline() as pipeline:

    # It's even possible to download a model for the used device
    # model_description = dai.NNModelDescription()
    # model_description.modelSlug = "ales-test"

    #  The two are equivalent
    # model_description.platform = pipeline.getDefaultDevice().getPlatformAsString()
    # model_description = dai.platform2string(pipeline.getDefaultDevice().getPlatform())

    # Color camera node
    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(256, 256)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
    camRgb.setInterleaved(False)

    # Neural network node
    neuralNetwork = pipeline.createNeuralNetwork()
    neuralNetwork.setBlobPath(model_path)
    neuralNetwork.setNumInferenceThreads(2)

    # Linking
    camRgb.preview.link(neuralNetwork.input)

    nnDetectionQueue = neuralNetwork.out.createQueue()
    nnPassthroughQueue = neuralNetwork.passthrough.createQueue()

    pipeline.start()

    while pipeline.isRunning():
        in_nn = nnDetectionQueue.get()
        in_nnPassthrough = nnPassthroughQueue.get()

        # Do whatever is needed with the inference results
        print(in_nn)
        print(in_nnPassthrough)

        time.sleep(0.1)