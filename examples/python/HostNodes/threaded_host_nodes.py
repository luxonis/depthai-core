import depthai as dai
import time

class TestPassthrough(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()
        self.output = self.createOutput()

    def onStart(self):
        print("Hello, this is", __class__.__name__)

    def onStop(self):
        print("Goodbye from", __class__.__name__)
    
    def run(self):
        while self.isRunning():
            buffer = self.input.get()
            print("The passthrough node received a buffer!")
            self.output.send(buffer)

class TestSink(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.input = self.createInput()

    def onStart(self):
        print("Hello, this is", __class__.__name__)

    def run(self):
        while self.isRunning():
            buffer = self.input.get()
            print("The sync node received a buffer!")

class TestSource(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.output = self.createOutput()

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