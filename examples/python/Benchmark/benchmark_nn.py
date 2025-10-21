import depthai as dai
import numpy as np


# First prepare the model for benchmarking
device = dai.Device()
modelPath = dai.getModelFromZoo(dai.NNModelDescription("yolov6-nano", platform=device.getPlatformAsString()))
modelArhive = dai.NNArchive(modelPath)
inputSize = modelArhive.getInputSize()
type = modelArhive.getConfig().model.inputs[0].preprocessing.daiType

if type:
    try:
        frameType = getattr(dai.ImgFrame.Type, type)
    except AttributeError:
        type = None

if not type:
    if device.getPlatform() == dai.Platform.RVC2:
        frameType = dai.ImgFrame.Type.BGR888p
    else:
        frameType = dai.ImgFrame.Type.BGR888i


# Construct the input (white) image for benchmarking
img = np.ones((inputSize[1], inputSize[0], 3), np.uint8) * 255
inputFrame = dai.ImgFrame()
inputFrame.setCvFrame(img, frameType)

with dai.Pipeline(device) as p:
    benchmarkOut = p.create(dai.node.BenchmarkOut)
    benchmarkOut.setRunOnHost(False) # The node can run on host or on device
    benchmarkOut.setFps(-1) # As fast as possible

    neuralNetwork = p.create(dai.node.NeuralNetwork).build(benchmarkOut.out, modelArhive)

    benchmarkIn = p.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(False) # The node can run on host or on device
    benchmarkIn.sendReportEveryNMessages(100)
    benchmarkIn.logReportsAsWarnings(False)
    neuralNetwork.out.link(benchmarkIn.input)

    outputQueue = benchmarkIn.report.createOutputQueue()
    inputQueue = benchmarkOut.input.createInputQueue()

    p.start()
    inputQueue.send(inputFrame) # Send the input image only once
    while p.isRunning():
        benchmarkReport = outputQueue.get()
        assert isinstance(benchmarkReport, dai.BenchmarkReport)
        print(f"FPS is {benchmarkReport.fps}")
