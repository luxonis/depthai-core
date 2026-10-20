#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


def main() -> None:
    with dai.Pipeline() as pipeline:
        modelSlug = "luxonis/paddle-text-detection:256x256"
        modelDescription = dai.NNModelDescription(
            modelSlug,
            platform=pipeline.getDefaultDevice().getPlatformAsString(),
        )
        modelArchive = dai.NNArchive(dai.getModelFromZoo(modelDescription))

        cameraNode = pipeline.create(dai.node.Camera).build()
        neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelArchive)
        parserNode = pipeline.create(dai.beta.node.PPTextDetectionParser).build(
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
            for detection in parserOutput.detections:
                boundingBox = detection.getBoundingBox().denormalize(
                    frame.shape[1], frame.shape[0]
                )
                points = np.array(
                    [(point.x, point.y) for point in boundingBox.getPoints()],
                    dtype=np.int32,
                )
                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                label = detection.labelName or str(detection.label)
                cv2.putText(
                    frame,
                    f"{label}: {detection.confidence:.2f}",
                    tuple(points[0]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )
                for point in detection.getKeypoints2f():
                    cv2.circle(
                        frame,
                        (int(point.x * frame.shape[1]), int(point.y * frame.shape[0])),
                        3,
                        (0, 0, 255),
                        -1,
                    )


            cv2.imshow("PPTextDetectionParser", frame)
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
