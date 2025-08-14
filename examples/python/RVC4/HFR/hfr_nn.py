#!/usr/bin/env python3
import depthai as dai

FPS = 480

with dai.Pipeline() as pipeline:
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
