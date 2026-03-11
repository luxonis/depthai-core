#!/usr/bin/env python3
import depthai as dai
import sys

FPS = 480

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
            if config.width == 1280 and config.height == 720 and config.maxFps >= FPS:
                supportsRequestedFps = True
                break
        break
    if not supportsRequestedFps:
        print("This example is only supported on IMX586 and Luxonis OS 1.20.5 or higher", file=sys.stderr)
        sys.exit(0)

    cameraNode = pipeline.create(dai.node.Camera).build(sensorFps=FPS)

    # Configure the DetectionNetwork
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription("yolov6-nano"))

    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)
    benchmarkIn.sendReportEveryNMessages(FPS)
    detectionNetwork.out.link(benchmarkIn.input)


    qDet = detectionNetwork.out.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        inDet: dai.ImgDetections = qDet.get()
        # print(f"Got {len(inDet.detections)} nn detections ")
