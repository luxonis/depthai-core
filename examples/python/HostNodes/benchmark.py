import depthai as dai
import time

class TestSource(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.output = self.createOutput()

    def run(self):
        while self.isRunning():
            buffer = dai.Buffer()
            print(f"{self.name} node is sending a buffer!")
            self.output.send(buffer)
            time.sleep(1)

with dai.Pipeline(createImplicitDevice=False) as p:
    # Create nodes
    source = TestSource("source")
    benchmarkIn = p.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)
    benchmarkIn.sendReportEveryNMessages(100)
    benchmarkOut = p.create(dai.node.BenchmarkOut)
    benchmarkOut.setRunOnHost(True)
    benchmarkOut.setFps(30)
    benchmarkOut.out.link(benchmarkIn.input)
    outputQueue = benchmarkIn.report.createOutputQueue()
    source.output.link(benchmarkOut.input)

    p.start()
    while p.isRunning():
        benchmarkReport = outputQueue.get()
        assert isinstance(benchmarkReport, dai.BenchmarkReport)
        print(f"FPS is {benchmarkReport.fps}")