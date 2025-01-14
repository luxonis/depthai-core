import depthai as dai
import time
import cv2

# Create pipeline
device = dai.Device()
pipeline = dai.Pipeline(device)

cameras = [
    pipeline.create(dai.node.Camera).build(socket)
    for socket in device.getConnectedCameras()
]
inputKeys = []
script = pipeline.create(dai.node.Script)

for i, camera in enumerate(cameras):
    inputName = str(i)
    camera.requestFullResolutionOutput().link(script.inputs[inputName])
    script.inputs[inputName].setBlocking(False)
    script.inputs[inputName].setMaxSize(1)
    inputKeys.append(inputName)

controlQueue = script.inputs["control"].createInputQueue()
preview = script.outputs["out"].createOutputQueue()

inputKeysSize = len(inputKeys)
script.setScript(
    f"""
    inputToStream = 0
    maxID = {inputKeysSize} - 1
    while True:
        controlMessage = node.inputs["control"].tryGet()
        if controlMessage is not None:
            if(inputToStream < maxID):
                inputToStream += 1
            else:
                inputToStream = 0
        frame = node.inputs[str(inputToStream)].get()
        node.outputs["out"].send(frame)
"""
)

pipeline.start()
print("To switch between streams, press 's'")
with pipeline:
    while pipeline.isRunning():
        previewMessage = preview.get()
        assert isinstance(previewMessage, dai.ImgFrame)
        cv2.imshow("preview", previewMessage.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord("s"):
            controlQueue.send(dai.Buffer())
        if key == ord("q"):
            break
