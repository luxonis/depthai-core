import depthai as dai

device= dai.Device()

with dai.Pipeline(device) as p:
    # Create a BenchmarkOut node
    # It will listen on the input to get the first message and then send it out at a specified rate
    # The node sends the same message out (creates new pointers), not deep copies.
    benchmarkOut = p.create(dai.node.BenchmarkOut)
    benchmarkOut.setRunOnHost(True) # The node can run on host or on device
    benchmarkOut.setFps(30)

    # Create a BenchmarkIn node
    # This node is receiving the messages on the input and measuring the FPS and latency.
    # In the case that the input is with BenchmarkOut, the latency measurement is not always possible, as the message is not deep copied,
    # which means that the timestamps stay the same and latency virtually increases over time.
    benchmarkIn = p.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(False) # The node can run on host or on device
    benchmarkIn.sendReportEveryNMessages(100)
    benchmarkIn.logReportsAsWarnings(False)
    benchmarkIn.setLogLevel(dai.LogLevel.TRACE)

    benchmarkOut.out.link(benchmarkIn.input)
    outputQueue = benchmarkIn.report.createOutputQueue()
    inputQueue = benchmarkOut.input.createInputQueue()

    p.start()
    imgFrame = dai.ImgFrame()
    inputQueue.send(imgFrame)
    while p.isRunning():
        benchmarkReport = outputQueue.get()
        assert isinstance(benchmarkReport, dai.BenchmarkReport)
        print(f"FPS is {benchmarkReport.fps}")
