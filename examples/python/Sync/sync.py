import depthai as dai

pipeline = dai.Pipeline()
left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)


sync = pipeline.create(dai.node.Sync)
sync.setRunOnHost(True) # Can also run on device
left.requestFullResolutionOutput().link(sync.inputs["left"])
right.requestFullResolutionOutput().link(sync.inputs["right"])

outQueue = sync.out.createOutputQueue()
pipeline.start()


while pipeline.isRunning():
    messageGroup : dai.MessageGroup = outQueue.get()
    left = messageGroup["left"]
    right = messageGroup["right"]
    print(f"Timestamps, message group {messageGroup.getTimestamp()}, left {left.getTimestamp()}, right {right.getTimestamp()}")