#!/usr/bin/env python3

import time

import cv2
import depthai as dai
import numpy as np

def showColoredSegmentationFrame(segMask: dai.SegmentationMask, colorScalingFactor: float):
    segFrame = segMask.getCvMask()
    segFrame = segFrame.astype(np.uint8)
    segFrame[segFrame < 255] = (segFrame[segFrame < 255] * colorScalingFactor).astype(np.uint8)
    coloredSegFrame = cv2.applyColorMap(segFrame, cv2.COLORMAP_JET)
    coloredSegFrame[segFrame == 255] = 0
    uniqueLabelIndexes = segMask.getUniqueIndices()
    if len(segMask.getLabels()) > 0:
        for i, labelIndex in enumerate(uniqueLabelIndexes):
            labelStr = segMask.getLabels()[labelIndex]
            color_value = np.array([[int(labelIndex * colorScalingFactor)]], dtype=np.uint8)
            text_color = cv2.applyColorMap(color_value, cv2.COLORMAP_JET)[0, 0].tolist()
            cv2.putText(coloredSegFrame, f"{labelIndex}: {labelStr}", (10, 20 + i * 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, text_color, 1)
    cv2.imshow("Segmentation Mask", coloredSegFrame)
    return coloredSegFrame

device = dai.Device()
modelName = "luxonis/deeplab-v3-plus:512x512"
if device.getPlatform() == dai.Platform.RVC2:
    modelName = "luxonis/deeplab-v3-plus:person-256x256"

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    cameraNode = pipeline.create(dai.node.Camera).build()
    neuralNetwork = pipeline.create(dai.node.NeuralNetwork).build(cameraNode, modelName)

    segParser = pipeline.create(dai.node.SegmentationParser).build(neuralNetwork.out, modelName)
    segParser.initialConfig.setConfidenceThreshold(0.0)

    maskQueue = segParser.out.createOutputQueue()
    frameQueue = neuralNetwork.passthrough.createOutputQueue()
    configQueue = segParser.inputConfig.createInputQueue()
    config = segParser.initialConfig
    colorScalingFactor = 255 / (len(segParser.getLabels()) + 1)

    pipeline.start()
    print("Pipeline created.")
    print("Controls: 'l' increase threshold, 'j' decrease threshold, 'q' quit.")

    start_time = time.monotonic()
    frames = 0

    while pipeline.isRunning():
        outSegMask: dai.SegmentationMask = maskQueue.get()
        frameMsg: dai.ImgFrame = frameQueue.get()

        frame = frameMsg.getCvFrame()
        coloredSegFrame = np.zeros_like(frame)
        if outSegMask.hasValidMask():
            coloredSegFrame = showColoredSegmentationFrame(outSegMask, colorScalingFactor)

        if frame.shape[0] != coloredSegFrame.shape[0] or frame.shape[1] != coloredSegFrame.shape[1]:
            coloredSegFrame = cv2.resize(coloredSegFrame, (frame.shape[1], frame.shape[0]))

        copyColoredFrame = coloredSegFrame.copy()
        copyColoredFrame[copyColoredFrame == 255] = frame[copyColoredFrame == 255]
        frame = cv2.addWeighted(frame, 0.5, copyColoredFrame, 0.5, 0)
        frames += 1
        fps = frames / max(1e-6, time.monotonic() - start_time)
        cv2.putText(
            frame,
            f"NN fps: {fps:.2f}",
            (2, frame.shape[0] - 4),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.4,
            (255, 255, 255),
        )
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            pipeline.stop()
            break
        elif key == ord('l'):
            current = config.getConfidenceThreshold()
            config.setConfidenceThreshold(current + 0.1)
            configQueue.send(config)
            print(f"Increased confidence threshold to {config.getConfidenceThreshold():.2f}")
        elif key == ord('j'):
            current = config.getConfidenceThreshold()
            config.setConfidenceThreshold(current - 0.1)
            configQueue.send(config)
            print(f"Decreased confidence threshold to {config.getConfidenceThreshold():.2f}")
