#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

def unpack_raw10(raw_data, width, height):
    """
    Unpacks RAW10 data from DepthAI pipeline into a 16-bit grayscale array.
    :param raw_data: List of raw bytes from DepthAI (1D numpy array)
    :param width: Image width
    :param height: Image height
    :return: Unpacked 16-bit grayscale image
    """
    expected_size = (width * height * 10) // 8

    if len(raw_data) < expected_size:
        raise ValueError(f"Data too small: {len(raw_data)} bytes, expected {expected_size}")

    # Convert raw_data to numpy array
    packed_data = np.frombuffer(raw_data, dtype=np.uint8)

    # Reshape into groups of 5 bytes (4 pixels per group)
    packed_data = packed_data[:expected_size].reshape(-1, 5)
    unpacked_data = np.zeros((packed_data.shape[0], 4), dtype=np.uint16)

    # Extract 8 most significant bits
    unpacked_data[:, 0] = packed_data[:, 0].astype(np.uint16) << 2
    unpacked_data[:, 1] = packed_data[:, 1].astype(np.uint16) << 2
    unpacked_data[:, 2] = packed_data[:, 2].astype(np.uint16) << 2
    unpacked_data[:, 3] = packed_data[:, 3].astype(np.uint16) << 2

    # Extract least significant 2 bits from 5th byte
    unpacked_data[:, 0] |= (packed_data[:, 4] & 0b00000011)
    unpacked_data[:, 1] |= (packed_data[:, 4] & 0b00001100) >> 2
    unpacked_data[:, 2] |= (packed_data[:, 4] & 0b00110000) >> 4
    unpacked_data[:, 3] |= (packed_data[:, 4] & 0b11000000) >> 6

    # Reshape to image dimensions
    raw_image = unpacked_data.flatten().reshape(height, width)

    # Scale from 10-bit (0-1023) to 16-bit (0-65535) for proper display
    raw_image_16bit = (raw_image * 64).astype(np.uint16)

    return raw_image_16bit

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    videoQueue = cam.raw.createOutputQueue()

    # Connect to device and start pipeline
    pipeline.start()
    while pipeline.isRunning():
        # videoIn = videoQueue2.tryGet()
        videoIn = None
        raw = videoQueue.tryGet()
        if raw is not None:
            assert isinstance(raw, dai.ImgFrame)
            data_raw = raw.getData()
            print(raw.getStride())
            raw_image = unpack_raw10(data_raw, raw.getStride(), raw.getHeight())
            cv2.imshow("raw", raw_image)
        if videoIn is not None:
            assert isinstance(videoIn, dai.ImgFrame)
            cv2.imshow("video", videoIn.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
