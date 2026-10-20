#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/ultra-fast-lane-detection:culane-800x288"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.LaneDetectionParser).build(
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
            for cluster in parserOutput.clusters:
                points = np.array(
                    [
                        (point.x * frame.shape[1], point.y * frame.shape[0])
                        for point in cluster.points
                    ],
                    dtype=np.int32,
                )
                if len(points) > 1:
                    cv2.polylines(frame, [points], False, (0, 255, 0), 3)

            cv2.imshow("LaneDetectionParser", frame)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
