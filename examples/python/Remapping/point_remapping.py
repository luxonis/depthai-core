#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai

SOURCE_WINDOW = "Source window (left)"
DEPTH_WINDOW = "Depth window (remapped)"
RGB_WINDOW = "RGB window (remapped)"

selectedPoint = None

def onLeftClick(event, x, y, flags, param):
    del flags, param
    global selectedPoint
    if event == cv2.EVENT_LBUTTONDOWN:
        selectedPoint = (x, y)


def toColorFrame(frame):
    if len(frame.shape) == 3 and frame.shape[2] == 3:
        return frame
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)


def drawPoint(frame, point, label, color):
    if point is not None:
        intPoint = (int(round(point.x)), int(round(point.y)))
        cv2.drawMarker(frame, intPoint, color, markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
        cv2.circle(frame, intPoint, 6, color, 1)
        cv2.putText(frame, label, (intPoint[0] + 10, max(20, intPoint[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    else:
        cv2.putText(frame, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def sampleDepth(point, depthFrame, patchRadius=2):
    if point is None:
        return None, "Left click a point on the source image"

    x, y = point
    depthData = depthFrame.getFrame()
    height, width = depthData.shape[:2]
    if x < 0 or x >= width or y < 0 or y >= height:
        return None, "Selected point is outside the depth frame"

    xStart = max(0, x - patchRadius)
    xEnd = min(width, x + patchRadius + 1)
    yStart = max(0, y - patchRadius)
    yEnd = min(height, y + patchRadius + 1)
    depthPatch = depthData[yStart:yEnd, xStart:xEnd].astype(np.float32)
    validDepth = depthPatch[depthPatch > 0]
    if validDepth.size == 0:
        return None, "No valid depth at selected point"

    depthMm = float(np.median(validDepth))
    return depthMm, f"z={depthMm:.0f}mm"

if __name__ == "__main__":
    pipeline = dai.Pipeline()

    colorSockets = pipeline.getDefaultDevice().getConnectedCameras(dai.CameraSensorType.COLOR)
    colorSocket = colorSockets[0] if colorSockets else dai.CameraBoardSocket.CAM_A
    rgb = pipeline.create(dai.node.Camera).build(colorSocket)

    stereoPair = pipeline.getDefaultDevice().getStereoPairs()[0]
    left = pipeline.create(dai.node.Camera).build(stereoPair.left)
    pipeline.create(dai.node.Camera).build(stereoPair.right)
    leftOut = left.requestFullResolutionOutput()

    rgbOut = rgb.requestOutput((720, 480), enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP)
    depth = pipeline.create(dai.node.Depth).build(dai.node.Depth.Algorithm.NEURAL)
    # depth.setConfig(dai.node.StereoDepth.PresetMode.DENSITY)

    rgbQueue = rgbOut.createOutputQueue()
    leftQueue = leftOut.createOutputQueue()
    depthQueue = depth.depth.createOutputQueue()

    cv2.namedWindow(SOURCE_WINDOW)
    cv2.namedWindow(DEPTH_WINDOW)
    cv2.namedWindow(RGB_WINDOW)
    cv2.setMouseCallback(SOURCE_WINDOW, onLeftClick)

    pipeline.start()
    while pipeline.isRunning():
        rgbFrame = rgbQueue.get()
        leftFrame = leftQueue.get()
        depthFrame = depthQueue.get()

        assert isinstance(rgbFrame, dai.ImgFrame)
        assert isinstance(leftFrame, dai.ImgFrame)
        assert isinstance(depthFrame, dai.ImgFrame)
        assert rgbFrame.validateTransformations()
        assert leftFrame.validateTransformations()
        assert depthFrame.validateTransformations()

        sourceTransformation = leftFrame.getTransformation()
        depthTransformation = depthFrame.getTransformation()
        rgbTransformation = rgbFrame.getTransformation()

        leftDisplay = toColorFrame(leftFrame.getCvFrame())
        rgbDisplay = rgbFrame.getCvFrame()
        depthColor = cv2.applyColorMap(cv2.convertScaleAbs(depthFrame.getFrame(), alpha=0.05), cv2.COLORMAP_JET)
        depthMm, depthStatus = sampleDepth(selectedPoint, depthFrame)

        remappedRgbPoint, remappedDepthPoint = None, None
        sourceStatus, rgbStatus, depthStatus = depthStatus, "", ""

        sourcePoint = None
        if selectedPoint is not None and depthMm is not None:
            sourcePoint = dai.Point2f(float(selectedPoint[0]), float(selectedPoint[1]))
            try:
                remappedRgbPoint = sourceTransformation.projectPointTo(rgbTransformation, sourcePoint, depthMm)
                remappedDepthPoint = sourceTransformation.projectPointTo(depthTransformation, sourcePoint, depthMm)
                sourceStatus = f"Source=({sourcePoint.x:.1f}, {sourcePoint.y:.1f}) z={depthMm:.0f}mm"
                rgbStatus = f"RGB=({remappedRgbPoint.x:.1f}, {remappedRgbPoint.y:.1f}) z={depthMm:.0f}mm"
                depthStatus = f"Depth=({remappedDepthPoint.x:.1f}, {remappedDepthPoint.y:.1f}) z={depthMm:.0f}mm"
            except RuntimeError as exc:
                rgbStatus = f"RGB projection failed: {exc}"

        drawPoint(leftDisplay, sourcePoint, sourceStatus, (0, 255, 0))
        drawPoint(depthColor, remappedDepthPoint, depthStatus, (0, 0, 255))
        drawPoint(rgbDisplay, remappedRgbPoint, f"{rgbStatus}", (255, 255, 0))

        cv2.imshow(SOURCE_WINDOW, leftDisplay)
        cv2.imshow(DEPTH_WINDOW, depthColor)
        cv2.imshow(RGB_WINDOW, rgbDisplay)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
        if key == ord('c'):
            selectedPoint = None
