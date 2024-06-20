#!/usr/bin/env python3

import depthai as dai

model_description = dai.NNModelDescription.fromParameters(modelSlug="ales-test", platform="rvc2")
print("Using model description:")
print(model_description)

model = dai.getModelFromZoo(model_description)