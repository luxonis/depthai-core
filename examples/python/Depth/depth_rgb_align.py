#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai
import time


class FPSCounter:
    def __init__(self):
        self.frameTimes = []

    def tick(self):
        now = time.time()
        self.frameTimes.append(now)
        self.frameTimes = self.frameTimes[-10:]

    def getFps(self):
        if len(self.frameTimes) <= 1:
            return 0
        return (len(self.frameTimes) - 1) / (self.frameTimes[-1] - self.frameTimes[0])


pipeline = dai.Pipeline()
device = pipeline.getDefaultDevice()

# Find a color camera automatically. Pick the first connected camera that
# advertises a COLOR sensor instead of hardcoding a socket.
colorSocket = None
for features in device.getConnectedCameraFeatures():
    if dai.CameraSensorType.COLOR in features.supportedTypes:
        colorSocket = features.socket
        break
if colorSocket is None:
    raise RuntimeError("No color camera found on this device to align the depth to.")
print(f"Aligning depth to color camera on socket: {colorSocket}")

# Unified Depth node (AUTO backend selection). The Depth node manages its own
# stereo backend and stereo cameras, resolving the algorithm at wiring time.
depthNode = pipeline.create(dai.node.Depth)

camRgb = pipeline.create(dai.node.Camera).build(colorSocket)
sync = pipeline.create(dai.node.Sync)

rgbOut = camRgb.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)

# Align depth to the color camera. The Depth node wires the alignment internally
# (StereoDepth's native alignTo or an ImageAlign node) depending on the platform
# and the resolved backend, so no ImageAlign node is needed.
depthNode.setAlignTo(rgbOut)
requestedOutput = depthNode.requestOutput()

rgbOut.link(sync.inputs["rgb"])
requestedOutput.depth.link(sync.inputs["depth_aligned"])

queue = sync.out.createOutputQueue()


def colorizeDepth(frameDepth):
    invalidMask = frameDepth == 0
    # Log the depth, minDepth and maxDepth
    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.zeros_like(frameDepth, dtype=np.float32)
        np.log(frameDepth, where=frameDepth != 0, out=logDepth)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        # Clip the values to be in the 0-255 range
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)

        # Interpolate only valid logDepth values, setting the rest based on the mask
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        # Set invalid depth pixels to black
        depthFrameColor[invalidMask] = 0
    except IndexError:
        # Frame is likely empty
        depthFrameColor = np.zeros((frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8)
    except Exception as e:
        raise e
    return depthFrameColor


rgbWeight = 0.4
depthWeight = 0.6


def updateBlendWeights(percentRgb):
    """
    Update the rgb and depth weights used to blend depth/rgb image

    @param[in] percent_rgb The rgb weight expressed as a percentage (0..100)
    """
    global depthWeight
    global rgbWeight
    rgbWeight = float(percentRgb) / 100.0
    depthWeight = 1.0 - rgbWeight


# Connect to device and start pipeline
with pipeline:
    pipeline.build()
    print(f"Resolved algorithm: {depthNode.getResolvedAlgorithm()}")
    print(f"Resolved config:    {depthNode.getResolvedConfig()}")
    pipeline.start()

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    windowName = "rgb-depth"

    # Set the window to be resizable and the initial size
    cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(windowName, 800, 600)
    cv2.createTrackbar(
        "RGB Weight %",
        windowName,
        int(rgbWeight * 100),
        100,
        updateBlendWeights,
    )
    fpsCounter = FPSCounter()
    while True:
        messageGroup = queue.get()
        fpsCounter.tick()
        assert isinstance(messageGroup, dai.MessageGroup)
        frameRgb = messageGroup["rgb"]
        assert isinstance(frameRgb, dai.ImgFrame)
        frameDepth = messageGroup["depth_aligned"]
        assert isinstance(frameDepth, dai.ImgFrame)

        # Blend when both received
        if frameDepth is not None:
            cvFrame = frameRgb.getCvFrame()
            # Colorize the aligned depth
            alignedDepthColorized = colorizeDepth(frameDepth.getFrame())
            cv2.imshow("Depth aligned", alignedDepthColorized)

            if len(cvFrame.shape) == 2:
                cvFrame = cv2.cvtColor(cvFrame, cv2.COLOR_GRAY2BGR)
            blended = cv2.addWeighted(
                cvFrame, rgbWeight, alignedDepthColorized, depthWeight, 0
            )
            cv2.putText(
                blended,
                f"FPS: {fpsCounter.getFps():.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            cv2.imshow(windowName, blended)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
