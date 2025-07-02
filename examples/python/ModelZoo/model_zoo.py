#!/usr/bin/env python3

import time
import depthai as dai

# Describe the model I want to download - two options: in program or from yaml file

# Option 1: In program
# modelDescription = dai.NNModelDescription(model="yolov6-nano", platform="RVC2", ...)

# Option 2: From yaml file
modelDescription = dai.NNModelDescription.fromYamlFile("./mymodel.yaml")

# If you want, you can store the model description in a yaml file
modelDescription.saveToYamlFile("./mymodel.yaml")

# Return path to downloaded model - yolov6-nano-r2-288x512.tar.xz for this example
modelPath = dai.getModelFromZoo(modelDescription, useCached=False, progressFormat="pretty")
print(f"Model path: {modelPath}")

# Load the model (most of the time it's a NNArchive)
archive = dai.NNArchive(modelPath)

print(f"Input size of the model {archive.getInputSize()}, name is {archive.getConfig().model.metadata.name}")
# The arhive can then be used with any of the neural network nodes (NeuralNetwork, DetectionNetwork, SpatialDetectionNetwork)
