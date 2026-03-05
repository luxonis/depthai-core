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

    # Download the model
    nnArchivePath = dai.getModelFromZoo(dai.NNModelDescription("yolov6-nano", platform="RVC4"))
    nnArchive = dai.NNArchive(nnArchivePath)
    inputSize = nnArchive.getInputSize()
    cameraNode = pipeline.create(dai.node.Camera).build()

    # Configure the ImageManip as in HFR mode requesting arbitrary outputs is not yet supported
    cameraOutput = cameraNode.requestOutput((1280, 720), fps=FPS)
    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setOutputSize(inputSize[0], inputSize[1])
    imageManip.setMaxOutputFrameSize(int(inputSize[0] * inputSize[1] * 3))
    imageManip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888i)
    imageManip.inputImage.setMaxSize(12)
    cameraOutput.link(imageManip.inputImage)

    # Configure the DetectionNetwork
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork)
    detectionNetwork.setNNArchive(nnArchive)
    imageManip.out.link(detectionNetwork.input)

    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)
    benchmarkIn.sendReportEveryNMessages(FPS)
    detectionNetwork.out.link(benchmarkIn.input)


    qDet = detectionNetwork.out.createOutputQueue()
    pipeline.start()

    while pipeline.isRunning():
        inDet: dai.ImgDetections = qDet.get()
        # print(f"Got {len(inDet.detections)} nn detections ")
