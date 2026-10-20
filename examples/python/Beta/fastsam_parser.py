#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/fastsam-s:512x288"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.FastSAMParser).build(
            neuralNetwork.out,
            modelArchive,
        )

        frameQueue = neuralNetwork.passthrough.createOutputQueue()
        outputQueue = parserNode.out.createOutputQueue()
        configQueue = parserNode.inputConfig.createInputQueue()
        config = parserNode.initialConfig

        pipeline.start()
        print("Controls: '+' increase confidence threshold, '-' decrease it, 'q' quit.")

        while pipeline.isRunning():
            frameMessage = frameQueue.get()
            parserOutput = outputQueue.get()
            frame = frameMessage.getCvFrame()
            mask = np.asarray(parserOutput.getCvMask(), dtype=np.uint8)
            mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
            coloredMask = cv2.applyColorMap(mask * 37, cv2.COLORMAP_TURBO)
            coloredMask[mask == 255] = 0
            frame = cv2.addWeighted(frame, 0.6, coloredMask, 0.4, 0)


            cv2.imshow("FastSAMParser", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("+"):
                config.confidenceThreshold = min(1.0, config.confidenceThreshold + 0.1)
                configQueue.send(config)
                print(f"Confidence threshold: {config.confidenceThreshold:.1f}")
            elif key == ord("-"):
                config.confidenceThreshold = max(0.0, config.confidenceThreshold - 0.1)
                configQueue.send(config)
                print(f"Confidence threshold: {config.confidenceThreshold:.1f}")


if __name__ == "__main__":
    main()
