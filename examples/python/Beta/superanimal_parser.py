#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/superanimal-landmarker:256x256"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.SuperAnimalParser).build(
            neuralNetwork.out,
            modelArchive,
        )

        frameQueue = neuralNetwork.passthrough.createOutputQueue()
        outputQueue = parserNode.out.createOutputQueue()
        configQueue = parserNode.inputConfig.createInputQueue()
        config = parserNode.initialConfig

        pipeline.start()
        print("Controls: '+' increase score threshold, '-' decrease it, 'q' quit.")

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


            cv2.imshow("SuperAnimalParser", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("+"):
                config.scoreThreshold = min(1.0, config.scoreThreshold + 0.1)
                configQueue.send(config)
                print(f"Score threshold: {config.scoreThreshold:.1f}")
            elif key == ord("-"):
                config.scoreThreshold = max(0.0, config.scoreThreshold - 0.1)
                configQueue.send(config)
                print(f"Score threshold: {config.scoreThreshold:.1f}")


if __name__ == "__main__":
    main()
