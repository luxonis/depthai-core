import depthai as dai
import numpy as np

COUNT = 3

def generate_random_grayscale_image(width, height):
    random_image = np.random.randint(0, 256, (height, width), dtype=np.uint8)
    return random_image

pipeline = dai.Pipeline()

script = pipeline.create(dai.node.Script)
script.setScript("""
while True:
    msggrp = node.io['in'].get()
    node.io['out'].send(msggrp)
""")

inq = script.inputs["in"].createInputQueue(maxSize = 100)

outq = script.outputs["out"].createOutputQueue()

pipeline.start()


for i in range(300):
    width = 1000
    height = 1000
    random_image = generate_random_grayscale_image(width, height)
    msgs = []
    for i in range(COUNT):
        img = dai.ImgFrame()
        img.setCvFrame(random_image, dai.ImgFrame.Type.GRAY8)
        msgs.append(img)

    messageGroup = dai.MessageGroup()
    for i, img in enumerate(msgs):
        messageGroup[f"img{i}"] = img

    inq.send(messageGroup)

    messageGroup : dai.MessageGroup = outq.get()
    print("Received message group")
