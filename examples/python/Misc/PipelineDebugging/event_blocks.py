import depthai as dai
import time

class Generator(dai.node.ThreadedHostNode):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.output1 = self.createOutput()
        self.output2 = self.createOutput()
        self.seq = 0

    def run(self):
        # Use self.mainLoop() to track main loop timings
        while self.mainLoop():
            buffer = dai.Buffer()
            buffer.setSequenceNum(self.seq)
            self.seq += 1
            # Track output send timings
            with self.outputBlockEvent():
                self.output1.send(buffer)
                self.output2.send(buffer)
            time.sleep(1)

with dai.Pipeline() as pipeline:
    # Pipeline debugging is disabled by default.
    # You can also enable it by setting the DEPTHAI_PIPELINE_DEBUGGING environment variable to '1'
    pipeline.enablePipelineDebugging(True)

    gen = Generator("Generator")
    script = pipeline.create(dai.node.Script)
    # Note: Using event blocks in the script node is only supported on RVC4
    script.setScript(
        """
        while node.mainLoop():
            with node.inputBlockEvent():
                message1 = node.inputs["in1"].get()
                message2 = node.inputs["in2"].get()
            with node.outputBlockEvent():
                node.outputs["out"].send(message1)
                node.outputs["out"].send(message2)
        """
    )

    gen.output1.link(script.inputs["in1"])
    gen.output2.link(script.inputs["in2"])
    outQ = script.outputs["out"].createOutputQueue()
    pipeline.start()
    i = 0
    while pipeline.isRunning() and i < 10:
        output = outQ.get()
        print(f"Received buffer with sequence number: {output.getSequenceNum()}")
        i += 1

    try:
        genState = pipeline.getPipelineState().nodes(gen.id).summary()
        scriptState = pipeline.getPipelineState().nodes(script.id).summary()
        print(f"\nGenerator node state:\n{genState}")
        print(f"\nScript node state:\n{scriptState}")
    except Exception as e:
        print(f"Could not get pipeline state: {e}")

