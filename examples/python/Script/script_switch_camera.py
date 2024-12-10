import depthai as dai
import time
import cv2

# Create pipeline
pipeline = dai.Pipeline()
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

script = pipeline.create(dai.node.Script)
left.requestFullResolutionOutput().link(script.inputs["left"])
right.requestFullResolutionOutput().link(script.inputs["right"])

# Don't block the script for the queue that's not getting streamed
script.inputs["left"].setBlocking(False)
script.inputs["left"].setMaxSize(1)
script.inputs["right"].setBlocking(False)
script.inputs["right"].setMaxSize(1)

controlQueue = script.inputs["control"].createInputQueue()
preview = script.outputs["out"].createOutputQueue()


script.setScript("""
    left = True
    while True:
        controlMessage = node.inputs["control"].tryGet()
        if controlMessage is not None:
            left = not left
        if left:
            frame = node.inputs["left"].get()
        else:
            frame = node.inputs["right"].get()
        node.outputs["out"].send(frame)
""")

pipeline.start()
print("To switch between left and right cameras, press 's'")
with pipeline:
    while pipeline.isRunning():
        previewMessage = preview.get()
        assert(isinstance(previewMessage, dai.ImgFrame))
        cv2.imshow("preview", previewMessage.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('s'):
            controlQueue.send(dai.Buffer())
        if key == ord('q'):
            break
