import depthai as dai
import time

class TestPassthrough(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.input = self.createInput()
        self.output = self.createOutput()

        # Possible API 1:
        self.input.setPossibleDatatypes([dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])
        self.output.setPossibleDatatypes([dai.Node.DatatypeHierarchy(dai.DatatypeEnum.ImgFrame, True)])

        # Possible API 2:
        self.input.setPossibleDatatypes([
            (dai.DatatypeEnum.ImgFrame, True),
            (dai.DatatypeEnum.Buffer, True)
        ])
        self.output.setPossibleDatatypes([
            (dai.DatatypeEnum.ImgFrame, True),
            (dai.DatatypeEnum.Buffer, True)
        ])



    def onStart(self):
        print("Hello, this is", self.name)

    def onStop(self):
        print("Goodbye from", self.name)

    def run(self):
        while self.mainLoop():
            buffer = self.input.get()
            print("The passthrough node received a buffer!")
            self.output.send(buffer)

class TestSink(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.input = self.createInput()

        self.name = name

    def onStart(self):
        print("Hello, this is", self.name)

    def run(self):
        while self.mainLoop():
            buffer = self.input.get()
            del buffer
            print(f"{self.name} node received a buffer!")

class TestSource(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.output = self.createOutput()

    def run(self):
        while self.mainLoop():
            buffer = dai.Buffer()
            print(f"{self.name} node is sending a buffer!")
            self.output.send(buffer)
            time.sleep(1)

class TestPassthroughSubnodes(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.passthrough1 = self.createSubnode(TestPassthrough, name + "_subnode1")
        self.passthrough2 = self.createSubnode(TestPassthrough, name + "_subnode2")
        self.input = self.passthrough1.input
        self.output = self.passthrough2.output

        # Link the two subnodes together
        self.passthrough1.output.link(self.passthrough2.input)

    def run(self):
        while self.mainLoop():
            buffer = self.input.get()
            self.output.send(buffer)

with dai.Pipeline(False) as p:
    """
    Create the following pipeline:
    source -> passthrough -> sink1
           |
           -> passthroughSubnodes -> sink2
    """

    # Create nodes
    source = TestSource("source")
    passthrough = TestPassthrough("passthrough")
    passthroughSubnodes = TestPassthroughSubnodes("passthroughSubnodes")
    sink1 = TestSink("sink1")
    sink2 = TestSink("sink2")

    # Link nodes
    source.output.link(passthrough.input)
    source.output.link(passthroughSubnodes.input)
    passthrough.output.link(sink1.input)
    passthroughSubnodes.output.link(sink2.input)

    p.start()
    while p.isRunning():
        time.sleep(1)
        print("Pipeline is running...")
