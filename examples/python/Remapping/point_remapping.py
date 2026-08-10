#!/usr/bin/env python3

import cv2
import numpy as np
import depthai as dai

SOURCE_WINDOW = "Source window CAM_B"
RGB_WINDOW = "RGB window CAM_A"

selectedPoint = None

def onLeftClick(event, x, y, flags, param):
    del flags, param
    global selectedPoint
    if event == cv2.EVENT_LBUTTONDOWN:
        selectedPoint = (x, y)

def toColorFrame(frame):
    if(len(frame.shape) == 3 and frame.shape[2] == 3):
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
        return None, "Left click to select a point"

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

    rgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()
    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)
    stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)

    rgbOut = rgb.requestOutput((720, 480), enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP)
    rgbQueue = rgbOut.createOutputQueue()
    depthQueue = stereo.depth.createOutputQueue()
    rectifiedLeftQueue = stereo.rectifiedLeft.createOutputQueue()

    cv2.namedWindow(SOURCE_WINDOW)
    cv2.namedWindow(RGB_WINDOW)
    cv2.setMouseCallback(SOURCE_WINDOW, onLeftClick)

    pipeline.start()
    while pipeline.isRunning():
        rgbFrame = rgbQueue.get()
        depthFrame = depthQueue.get()
        rectifiedLeft = rectifiedLeftQueue.get()

        assert isinstance(rgbFrame, dai.ImgFrame)
        assert isinstance(depthFrame, dai.ImgFrame)
        assert isinstance(rectifiedLeft, dai.ImgFrame)
        assert rgbFrame.validateTransformations()
        assert depthFrame.validateTransformations()
        assert rectifiedLeft.validateTransformations()

        sourceTransformation = rectifiedLeft.getTransformation()
        rgbTransformation = rgbFrame.getTransformation()
        depthTransformation = depthFrame.getTransformation()

        leftFrame = toColorFrame(rectifiedLeft.getCvFrame())
        rgbDisplay = rgbFrame.getCvFrame()
        depthMm, sourceStatus = sampleDepth(selectedPoint, depthFrame)

        remappedRgbPoint, remappedDepthPoint = None, None
        rgbStatus, depthStatus = "", ""

        originalPoint = None
        if selectedPoint is not None and depthMm is not None:
            sourcePoint = dai.Point2f(float(selectedPoint[0]), float(selectedPoint[1]))
            originalPoint = sourcePoint
            try:
                remappedRgbPoint = sourceTransformation.projectPointTo(rgbTransformation, sourcePoint, depthMm)
                remappedDepthPoint = sourceTransformation.projectPointTo(depthTransformation, sourcePoint, depthMm)
                rgbStatus = f"RGB=({remappedRgbPoint.x:.1f}, {remappedRgbPoint.y:.1f}) z={depthMm:.0f}mm"
                sourceStatus = f"Source=({sourcePoint.x:.1f}, {sourcePoint.y:.1f}) z={depthMm:.0f}mm"
                depthStatus = f"Depth=({remappedDepthPoint.x:.1f}, {remappedDepthPoint.y:.1f}) z={depthMm:.0f}mm"

            except RuntimeError as exc:
                rightStatus = f"R projection failed: {exc}"
                rgbStatus = f"RGB projection failed: {exc}"

        depthColor = cv2.applyColorMap(cv2.convertScaleAbs(depthFrame.getFrame(), alpha=0.05), cv2.COLORMAP_JET)
        drawPoint(leftFrame, originalPoint, f"{sourceStatus}", (0, 255, 0))
        drawPoint(rgbDisplay, remappedRgbPoint, f"{rgbStatus}", (255, 255, 0))
        drawPoint(depthColor, remappedDepthPoint, f"{depthStatus}", (0, 0, 255))

        cv2.imshow("Depth", depthColor)
        cv2.imshow(SOURCE_WINDOW, leftFrame)
        cv2.imshow(RGB_WINDOW, rgbDisplay)
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
        if key == ord('c'):
            selectedPoint = None
