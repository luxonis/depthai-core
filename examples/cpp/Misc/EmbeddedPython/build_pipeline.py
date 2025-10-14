import depthai as dai

def build_fn(p: dai.Pipeline):
    # Build a whole complicated pipeline here, for the example just build a camera node
    cam = p.create(dai.node.Camera).build()
    return cam.requestFullResolutionOutput().createOutputQueue()

class PyHostPassthrough(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.name = "PyHostPassthrough"
        self.input = self.createInput("py_input")
        self.output = self.createOutput("py_output")

        self.input.setPossibleDatatypes([
            (dai.DatatypeEnum.ImgFrame, True),
            (dai.DatatypeEnum.Buffer, True)
        ])
        self.output.setPossibleDatatypes([
            (dai.DatatypeEnum.ImgFrame, True),
            (dai.DatatypeEnum.Buffer, True)
        ])
    def run(self):
        while self.isRunning():
            buffer = self.input.get()
            print("The passthrough node received a buffer!")
            self.output.send(buffer)


def build_with_py_nodes(p: dai.Pipeline):
    cam = p.create(dai.node.Camera).build()
    py_node = p.create(PyHostPassthrough)
    cam.requestFullResolutionOutput().link(py_node.input)
    return py_node.output.createOutputQueue()

