#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/emotion-recognition:260x260"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.ClassificationParser).build(
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
            for index, (label, score) in enumerate(
                zip(parserOutput.classes[:5], parserOutput.scores[:5])
            ):
                cv2.putText(
                    frame,
                    f"{label}: {score:.2f}",
                    (20, 35 + index * 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 0),
                    2,
                )

            cv2.imshow("ClassificationParser", frame)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
