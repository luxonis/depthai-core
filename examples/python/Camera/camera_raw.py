#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def unpack_raw10(rawData, width, height):
    """
    Unpacks RAW10 data from DepthAI pipeline into a 16-bit grayscale array.
    :param raw_data: List of raw bytes from DepthAI (1D numpy array)
    :param width: Image width
    :param height: Image height
    :return: Unpacked 16-bit grayscale image
    """
    expectedSize = (width * height * 10) // 8

    if len(rawData) < expectedSize:
        raise ValueError(f"Data too small: {len(rawData)} bytes, expected {expectedSize}")

    # Convert raw_data to numpy array
    packedData = np.frombuffer(rawData, dtype=np.uint8)

    # Reshape into groups of 5 bytes (4 pixels per group)
    packedData = packedData[:expectedSize].reshape(-1, 5)
    unpackedData = np.zeros((packedData.shape[0], 4), dtype=np.uint16)

    # Extract 8 most significant bits
    unpackedData[:, 0] = packedData[:, 0].astype(np.uint16) << 2
    unpackedData[:, 1] = packedData[:, 1].astype(np.uint16) << 2
    unpackedData[:, 2] = packedData[:, 2].astype(np.uint16) << 2
    unpackedData[:, 3] = packedData[:, 3].astype(np.uint16) << 2

    # Extract least significant 2 bits from 5th byte
    unpackedData[:, 0] |= (packedData[:, 4] & 0b00000011)
    unpackedData[:, 1] |= (packedData[:, 4] & 0b00001100) >> 2
    unpackedData[:, 2] |= (packedData[:, 4] & 0b00110000) >> 4
    unpackedData[:, 3] |= (packedData[:, 4] & 0b11000000) >> 6

    # Reshape to image dimensions
    rawImage = unpackedData.flatten().reshape(height, width)

    # Scale from 10-bit (0-1023) to 16-bit (0-65535) for proper display
    rawImage16bit = (rawImage * 64).astype(np.uint16)

    return rawImage16bit

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
            parsedImage = unpack_raw10(dataRaw, rawFrame.getStride(), rawFrame.getHeight())
            cv2.imshow("raw", parsedImage)
        if videoIn is not None:
            assert isinstance(videoIn, dai.ImgFrame)
            cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
