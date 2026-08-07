#!/usr/bin/env python3

import cv2
import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/gaze-estimation-adas:60x60"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.RegressionParser).build(
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
            values = " ".join(
                f"{prediction.prediction:.3f}" for prediction in parserOutput.predictions
            )
            cv2.putText(
                frame,
                values,
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
            )

            cv2.imshow("RegressionParser", frame)
            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
