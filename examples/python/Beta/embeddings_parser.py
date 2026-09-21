#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/arcface:lfw-112x112"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.EmbeddingsParser).build(
            neuralNetwork.out,
            modelArchive,
        )

        frameQueue = neuralNetwork.passthrough.createOutputQueue()
        outputQueue = parserNode.out.createOutputQueue()

        pipeline.start()

        while pipeline.isRunning():
            frameMessage = frameQueue.get()
            parserOutput = outputQueue.get()
            frame = frameMessage.getCvFrame()
            layerName = parserOutput.getAllLayerNames()[0]
            embedding = parserOutput.getTensor(layerName, True).reshape(-1)
            cv2.putText(
                frame,
                f"Embedding size: {embedding.size}, norm: {np.linalg.norm(embedding):.2f}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.imshow("EmbeddingsParser", frame)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
