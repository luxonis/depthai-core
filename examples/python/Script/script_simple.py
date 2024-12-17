import depthai as dai
import time

# Create pipeline
pipeline = dai.Pipeline()
script = pipeline.create(dai.node.Script)
inputQueue = script.inputs["in"].createInputQueue()
outputQueue = script.outputs["out"].createOutputQueue()


script.setScript(
    """
    while True:
        message = node.inputs["in"].get()
        # Or alternatively:
        # message = node.io["in"].get()
        node.warn("I received a message!")
        node.outputs["out"].send(message)
        # Or alternatively:
        # node.io["out"].send(message)
"""
)

pipeline.start()
with pipeline:
    while pipeline.isRunning():
        message = dai.ImgFrame()
        print("Sending a message")
        inputQueue.send(message)
        output = outputQueue.get()
        print("Received a message")
        time.sleep(1)
