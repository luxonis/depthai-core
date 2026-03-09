#!/usr/bin/env python3
import depthai as dai
import sys
import time
import cv2

SIZE = (1280, 720)
FPS = 480

# SIZE = (1920, 1080)
# FPS = 240

with dai.Pipeline() as pipeline:
    device = pipeline.getDefaultDevice()
    platform = device.getPlatform()
    if platform != dai.Platform.RVC4:
        print("This example is only supported on IMX586 and Luxonis OS 1.20.5 or higher", file=sys.stderr)
        sys.exit(0)

    # Exit cleanly if the selected HFR mode is not advertised by CAM_A.
    supportsRequestedFps = False
    for cameraFeature in device.getConnectedCameraFeatures():
        if cameraFeature.socket != dai.CameraBoardSocket.CAM_A:
            continue
        for config in cameraFeature.configs:
            if config.width == SIZE[0] and config.height == SIZE[1] and config.maxFps >= FPS:
                supportsRequestedFps = True
                break
        break
    if not supportsRequestedFps:
        print("This example is only supported on IMX586 and Luxonis OS 1.20.5 or higher", file=sys.stderr)
        sys.exit(0)

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
