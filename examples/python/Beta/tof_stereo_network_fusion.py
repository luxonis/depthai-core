#!/usr/bin/env python3
"""ToF + stereo fusion with the STEREO_NETWORK model.

One network takes the rectified stereo pair and the ToF reprojected into the rectified-left view, and the device composes
its disparity with the ToF. The model is not in the model zoo. Models with a custom operator need its registration
library; see the NeuralNetwork backend properties udo_package and output_tensors.
"""

import argparse

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("model", help="network model (.dlc)")
parser.add_argument("--udo-package", default="", help="UDO registration library of the model")
parser.add_argument("--size", type=int, nargs=2, default=(1280, 800), metavar=("WIDTH", "HEIGHT"))
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=30)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=30)
    fusion = pipeline.create(dai.beta.node.ToFStereoFusion)
    fusion.setModel(dai.beta.node.ToFStereoFusion.Model.STEREO_NETWORK)
    fusion.setNetworkInputSize(*args.size)
    fusion.build(left, right)
    fusion.neuralNetwork.setModelPath(args.model)
    backend_properties = {"output_tensors": "disp_low,head9,conf", "output_memory": "dma_buf", "register_dma_buf": "true"}
    if args.udo_package:
        backend_properties["udo_package"] = args.udo_package
    fusion.neuralNetwork.setBackendProperties(backend_properties)
    # Keep pixels at depth Z with confidence >= 0.6 * Z / 10 m, and output the ToF depth closer than the network can.
    fusion.initialConfig.setConfidenceThreshold(0.6)
    fusion.initialConfig.setNearRangePassthrough(True)

    depth_queue = fusion.depth.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        depth = depth_queue.get()
        cv2.imshow("Fused depth (rectified left)", dai.utility.colorizeDepthFrame(depth).getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
