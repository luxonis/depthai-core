#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/mediapipe-face-landmarker:192x192"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.KeypointParser).build(
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
            points = [
                (int(point.x * frame.shape[1]), int(point.y * frame.shape[0]))
                for point in parserOutput.getPoints2f()
            ]
            for edge in parserOutput.getEdges():
                cv2.line(frame, points[edge[0]], points[edge[1]], (0, 255, 0), 2)
            for point in points:
                cv2.circle(frame, point, 3, (0, 0, 255), -1)

            cv2.imshow("KeypointParser", frame)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
