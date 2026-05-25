#!/usr/bin/env python3

import cv2
import depthai as dai

DISTORTED_WINDOW = "CAM_A distorted 640x480"
UNDISTORTED_WINDOW = "CAM_A undistorted 1000x400"
DISTORTED_SIZE = (640, 480)
UNDISTORTED_SIZE = (1000, 400)

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


def addStatusLines(frame, lines, color=(255, 0, 255)):
    for index, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (10, 28 + index * 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )


def isInsideFrame(point, frame):
    height, width = frame.shape[:2]
    return 0 <= point.x < width and 0 <= point.y < height


def drawPoint(frame, point, color):
    if point is None or not isInsideFrame(point, frame):
        return
    intPoint = (int(round(point.x)), int(round(point.y)))
    cv2.drawMarker(frame, intPoint, color, markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
    cv2.circle(frame, intPoint, 6, color, 1)


if __name__ == "__main__":
    with dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        distorted = camera.requestOutput(DISTORTED_SIZE, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=False)
        undistorted = camera.requestOutput(UNDISTORTED_SIZE, resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

        distQueue = distorted.createOutputQueue()
        undistQueue = undistorted.createOutputQueue()

        cv2.namedWindow(DISTORTED_WINDOW)
        cv2.namedWindow(UNDISTORTED_WINDOW)
        cv2.setMouseCallback(DISTORTED_WINDOW, onLeftClick)

        print("Left click in the distorted CAM_A window to remap a point to the undistorted output.")
        print("Press 'c' to clear the point and 'q' to quit.")

        pipeline.start()
        while pipeline.isRunning():
            distortedFrame = distQueue.get()
            undistortedFrame = undistQueue.get()
            assert isinstance(distortedFrame, dai.ImgFrame)
            assert isinstance(undistortedFrame, dai.ImgFrame)
            assert distortedFrame.validateTransformations()
            assert undistortedFrame.validateTransformations()

            distortedTransform = distortedFrame.getTransformation()
            undistortedTransform = undistortedFrame.getTransformation()

            distortedView = toColorFrame(distortedFrame.getCvFrame())
            undistortedView = toColorFrame(undistortedFrame.getCvFrame())

            sourcePoint = None
            remappedPoint = None
            sourceStatus = "Click in this window to select a point"
            targetStatus = "Waiting for a selected point"

            if selectedPoint is not None:
                sourcePoint = dai.Point2f(float(selectedPoint[0]), float(selectedPoint[1]))
                remappedPoint = distortedTransform.remapPointTo(undistortedTransform, sourcePoint)
                sourceStatus = f"Source: ({sourcePoint.x:.1f}, {sourcePoint.y:.1f})"
                targetStatus = f"Remapped: ({remappedPoint.x:.1f}, {remappedPoint.y:.1f})"
                if not isInsideFrame(remappedPoint, undistortedView):
                    targetStatus += " outside target frame"


            drawPoint(distortedView, sourcePoint, (0, 255, 0))
            drawPoint(undistortedView, remappedPoint, (0, 255, 255))

            addStatusLines(
                distortedView,
                [
                    "Distorted CAM_A output",
                    sourceStatus,
                    "Press c to clear, q to quit",
                ],
            )
            addStatusLines(
                undistortedView,
                [
                    "Undistorted CAM_A output",
                    targetStatus,
                    f"Target size: {UNDISTORTED_SIZE[0]}x{UNDISTORTED_SIZE[1]}",
                ],
            )

            cv2.imshow(DISTORTED_WINDOW, distortedView)
            cv2.imshow(UNDISTORTED_WINDOW, undistortedView)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            if key == ord("c"):
                selectedPoint = None
