#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/m-lsd:512x512"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.MLSDParser).build(
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
            for line in parserOutput.lines:
                startPoint = (
                    int(line.startPoint.x * frame.shape[1]),
                    int(line.startPoint.y * frame.shape[0]),
                )
                endPoint = (
                    int(line.endPoint.x * frame.shape[1]),
                    int(line.endPoint.y * frame.shape[0]),
                )
                cv2.line(frame, startPoint, endPoint, (0, 255, 0), 2)


            cv2.imshow("MLSDParser", frame)
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
