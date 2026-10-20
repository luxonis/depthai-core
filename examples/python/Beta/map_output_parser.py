#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/dm-count:sha-426x240"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.MapOutputParser).build(
            neuralNetwork.out,
            modelArchive,
        )

        frameQueue = neuralNetwork.passthrough.createOutputQueue()
        outputQueue = parserNode.out.createOutputQueue()
        configQueue = parserNode.inputConfig.createInputQueue()
        config = parserNode.initialConfig

        pipeline.start()
        print("Controls: 't' toggle min/max scaling, 'q' quit.")

        while pipeline.isRunning():
            frameMessage = frameQueue.get()
            parserOutput = outputQueue.get()
            frame = frameMessage.getCvFrame()
            mapValues = np.asarray(parserOutput.getMap(), dtype=np.float32)
            normalizedMap = cv2.normalize(mapValues, None, 0, 255, cv2.NORM_MINMAX)
            frame = cv2.applyColorMap(normalizedMap.astype(np.uint8), cv2.COLORMAP_INFERNO)
            frame = cv2.resize(frame, (frameMessage.getWidth(), frameMessage.getHeight()))


            cv2.imshow("MapOutputParser", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("t"):
                config.minMaxScaling = not config.minMaxScaling
                configQueue.send(config)
                print(f"Min/max scaling: {config.minMaxScaling}")


if __name__ == "__main__":
    main()
