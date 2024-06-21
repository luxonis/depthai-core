import depthai as dai
import time

class TestPassthrough(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.input = dai.Node.Input(self)
        self.output = dai.Node.Output(self)
    def run(self):
        while self.isRunning():
            buffer = self.input.get()
            print("The passthrough node received a buffer!")
            self.output.send(buffer)

class TestSink(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.input = dai.Node.Input(self)
    def run(self):
        while self.isRunning():
            buffer = self.input.get()
            print("The sync node received a buffer!")

class TestSource(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        while self.isRunning():
            buffer = dai.Buffer()
            print("The source node is sending a buffer!")
            self.output.send(buffer)
            time.sleep(1)

with dai.Pipeline(False) as p:
    source = TestSource()
    sink = TestSink()
    passthrough = TestPassthrough()
    source.output.link(passthrough.input)
    passthrough.output.link(sink.input)
    p.start()
    while True:
        time.sleep(1)
        print("Pipeline is running...")