#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/paddle-text-recognition:320x48"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.ClassificationSequenceParser).build(
            neuralNetwork.out,
            modelArchive,
        )

        frameQueue = neuralNetwork.passthrough.createOutputQueue()
        outputQueue = parserNode.out.createOutputQueue()
        configQueue = parserNode.inputConfig.createInputQueue()
        config = parserNode.initialConfig

        pipeline.start()
        print("Controls: 't' toggle remove duplicates, 'q' quit.")

        while pipeline.isRunning():
            frameMessage = frameQueue.get()
            parserOutput = outputQueue.get()
            frame = frameMessage.getCvFrame()
            separator = "" if all(len(label) <= 1 for label in parserOutput.classes) else " "
            decodedText = separator.join(parserOutput.classes)
            cv2.putText(
                frame,
                decodedText,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )


            cv2.imshow("ClassificationSequenceParser", frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("t"):
                config.removeDuplicates = not config.removeDuplicates
                configQueue.send(config)
                print(f"Remove duplicates: {config.removeDuplicates}")


if __name__ == "__main__":
    main()
