#!/usr/bin/env python3
import depthai as dai
import time
import cv2

SIZE = (1280, 720)
FPS = 480

# SIZE = (1920, 1080)
# FPS = 240
with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build()
    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)
    benchmarkIn.sendReportEveryNMessages(FPS)
    
    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setOutputSize(250, 250)
    imageManip.setMaxOutputFrameSize(int(250* 250 * 1.6))

    # One of the two modes can be selected
    # NOTE: Generic resolutions are not yet supported through camera node when using HFR mode
    output = cam.requestOutput(SIZE, fps=FPS)

    output.link(imageManip.inputImage)
    imageManip.out.link(benchmarkIn.input)

    outputQueue = imageManip.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        imgFrame = outputQueue.get()
        assert isinstance(imgFrame, dai.ImgFrame)
        cv2.imshow("frame", imgFrame.getCvFrame())
        cv2.waitKey(1)
