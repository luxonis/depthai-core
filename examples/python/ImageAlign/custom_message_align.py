#!/usr/bin/env python3

import cv2
import depthai as dai

FPS = 30.0
SourceSize = (640, 640)
AlignToSize = (1280, 720)

class Line(dai.TransformableBuffer):
    def __init__(self):
        super().__init__()
        self.startPoint: dai.Point2f = None
        self.endPoint: dai.Point2f = None

    # Need to override the base method to properly transform custom messages
    def transformTo(self, target: dai.ImgTransformation) -> dai.TransformableBuffer:
        source = self.getTransformation()
        assert source is not None, "Source transformation is not set for the line"

        out = Line()
        out.startPoint = source.remapPointTo(target, self.startPoint)
        out.endPoint = source.remapPointTo(target, self.endPoint)
        out.setTransformation(target)
        return out


def drawLine(frame, line: Line, color, label: str) -> None:
    start = (int(round(line.startPoint.x)), int(round(line.startPoint.y)))
    end = (int(round(line.endPoint.x)), int(round(line.endPoint.y)))

    cv2.line(frame, start, end, color, 1)
    cv2.circle(frame, start, 2, color, -1)
    cv2.circle(frame, end, 2, color, -1)
    cv2.putText(frame, label, (start[0] + 12, max(24, start[1] - 12)), cv2.FONT_HERSHEY_TRIPLEX, 0.6, color)


def makeLine(SourceFrame: dai.ImgFrame) -> Line:
    line = Line()
    line.setTransformation(SourceFrame.getTransformation())
    line.startPoint = dai.Point2f(110.0, 150.0)
    line.endPoint = dai.Point2f(520.0, 430.0)
    return line


if __name__ == "__main__":
    pipeline = dai.Pipeline()
    camera = pipeline.create(dai.node.Camera).build(boardSocket=dai.CameraBoardSocket.CAM_A, sensorFps=FPS)

    sourceOutput = camera.requestOutput(size=SourceSize, fps=FPS, resizeMode=dai.ImgResizeMode.LETTERBOX, enableUndistortion=False)
    alignToOutput = camera.requestOutput(size=AlignToSize, fps=FPS, resizeMode=dai.ImgResizeMode.STRETCH, enableUndistortion=True)

    align = pipeline.create(dai.node.ImageAlign)
    align.setRunOnHost(True) # for custom message, ImageAlign needs to run on host.
    alignToOutput.link(align.inputAlignTo)

    sourceQueue = sourceOutput.createOutputQueue()
    alignToQueue = alignToOutput.createOutputQueue()
    alignedLineQueue = align.outputAligned.createOutputQueue()

    lineInputQueue = align.input.createInputQueue()

    pipeline.start()

    while pipeline.isRunning():
        sourceFrameMsg: dai.ImgFrame = sourceQueue.get()
        alignToFrameMsg: dai.ImgFrame = alignToQueue.get()

        sourceLine = makeLine(sourceFrameMsg)
        lineInputQueue.send(sourceLine)
        alignedLine: Line = alignedLineQueue.get()
        assert isinstance(alignedLine, Line)

        sourceFrame = sourceFrameMsg.getCvFrame()
        alignToFrame = alignToFrameMsg.getCvFrame()

        drawLine(sourceFrame, sourceLine, (0, 255, 0), "source line")
        drawLine(alignToFrame, alignedLine, (0, 140, 255), "aligned line")

        cv2.putText(sourceFrame, "source: CROP 640x640", (10, 28), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))
        cv2.putText(alignToFrame, "alignTo: STRETCH 1280x720", (10, 28), cv2.FONT_HERSHEY_TRIPLEX, 0.6, (255, 255, 255))

        cv2.imshow("Source output", sourceFrame)
        cv2.imshow("Aligned output", alignToFrame)

        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
