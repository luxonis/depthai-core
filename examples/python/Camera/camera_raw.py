#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def unpackRaw10(rawData, width, height, stride=None):
    """
    Unpacks RAW10 data from DepthAI pipeline into a 16-bit grayscale array.
    :param raw_data: List of raw bytes from DepthAI (1D numpy array)
    :param width: Image width
    :param height: Image height
    :param stride: Row stride in bytes (if None, calculated as width*10/8)
    :return: Unpacked 16-bit grayscale image with dimensions width×height
    """
    if stride is None:
        stride = width * 10 // 8
    expectedSize = stride * height

    if len(rawData) < expectedSize:
        raise ValueError(f"Data too small: {len(rawData)} bytes, expected {expectedSize}")

    # Convert raw_data to numpy array
    packedData = np.frombuffer(rawData, dtype=np.uint8)

    # Process image row by row to handle stride correctly
    result = np.zeros((height, width), dtype=np.uint16)

    for row in range(height):
        # Get row data using stride
        rowStart = row * stride
        rowData = packedData[rowStart:rowStart + stride]
        # Calculate how many complete 5-byte groups we need for width pixels
        numGroups = (width + 3) // 4  # Ceiling division
        rowBytes = numGroups * 5
        # Ensure we don't go beyond available data
        if len(rowData) < rowBytes:
            break

        # Process only the bytes we need for this row
        rowPacked = rowData[:rowBytes].reshape(-1, 5)
        rowUnpacked = np.zeros((rowPacked.shape[0], 4), dtype=np.uint16)

        # Extract 8 most significant bits
        rowUnpacked[:, 0] = rowPacked[:, 0].astype(np.uint16) << 2
        rowUnpacked[:, 1] = rowPacked[:, 1].astype(np.uint16) << 2
        rowUnpacked[:, 2] = rowPacked[:, 2].astype(np.uint16) << 2
        rowUnpacked[:, 3] = rowPacked[:, 3].astype(np.uint16) << 2

        # Extract least significant 2 bits from 5th byte
        rowUnpacked[:, 0] |= (rowPacked[:, 4] & 0b00000011)
        rowUnpacked[:, 1] |= (rowPacked[:, 4] & 0b00001100) >> 2
        rowUnpacked[:, 2] |= (rowPacked[:, 4] & 0b00110000) >> 4
        rowUnpacked[:, 3] |= (rowPacked[:, 4] & 0b11000000) >> 6

        # Flatten and copy only the required width pixels to result
        rowFlat = rowUnpacked.flatten()
        result[row, :width] = rowFlat[:width]

    # Scale from 10-bit (0-1023) to 16-bit (0-65535) for proper display
    result16bit = (result * 64).astype(np.uint16)
    return result16bit

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rawQueue = cam.raw.createOutputQueue()
    videoQueue = cam.requestFullResolutionOutput().createOutputQueue()
    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        videoIn = videoQueue.tryGet()
        rawFrame = rawQueue.tryGet()
        if rawFrame is not None:
            assert isinstance(rawFrame, dai.ImgFrame)
            dataRaw = rawFrame.getData()
            parsedImage = unpackRaw10(dataRaw, rawFrame.getWidth(), rawFrame.getHeight(), rawFrame.getStride())
            cv2.imshow("raw", parsedImage)
        if videoIn is not None:
            assert isinstance(videoIn, dai.ImgFrame)
            cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
