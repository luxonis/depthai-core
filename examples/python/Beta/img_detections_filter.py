#!/usr/bin/env python3

import time

import depthai as dai


def make_detection(label: int, confidence: float) -> dai.ImgDetection:
    detection = dai.ImgDetection()
    detection.label = label
    detection.confidence = confidence
    return detection


def print_detections(detections: dai.ImgDetections) -> None:
    for detection in detections.detections:
        print(f"  class={detection.label}, confidence={detection.confidence:.2f}")


def main() -> None:
    device = dai.Device()
    with dai.Pipeline(device) as pipeline:
        detections_filter = pipeline.create(dai.beta.node.ImgDetectionsFilter)
        input_queue = detections_filter.input.createInputQueue()
        config_queue = detections_filter.inputConfig.createInputQueue()
        output_queue = detections_filter.output.createOutputQueue()

        pipeline.start()

        config = dai.beta.ImgDetectionsFilterConfig()
        config.confidenceThreshold = 0.7
        config_queue.send(config)

        sequence_num = 0

        try:
            while pipeline.isRunning():
                detections = dai.ImgDetections()
                detections.setSequenceNum(sequence_num)
                detections.detections = [
                    make_detection(label=0, confidence=0.95),
                    make_detection(label=1, confidence=0.60),
                ]

                input_queue.send(detections)
                forwarded = output_queue.get(timeout=1.0)

                if forwarded is None:
                    raise RuntimeError("Timed out waiting for ImgDetectionsFilter output")

                print(f"Sequence number: {forwarded.getSequenceNum()}")
                print_detections(forwarded)
                print()

                sequence_num += 1
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
