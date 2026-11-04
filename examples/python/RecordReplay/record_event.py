import argparse

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default=".", help="Directory that receives the recording")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    record = pipeline.create(dai.node.RecordVideo)
    record.setFormat(dai.RecordFormat.RAW_FRAMES)
    record.setOutputDirectory(args.output)
    record.setStreamName("event")
    record.setWaitForTrigger(True)
    record.setPreTriggerFrames(30)

    camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12).link(record.input)
    # RecordControl is built on the host. A Script node on the device cannot construct this
    # message until the device firmware accepts RecordControl.
    control = record.inputControl.createInputQueue()
    status = record.status.createOutputQueue(16, True)

    pipeline.start()
    input("Waiting. The last 30 frames are kept. Press Enter to record them and what follows.\n")
    control.send(dai.RecordControl.start())
    input("Recording. Press Enter to stop.\n")
    control.send(dai.RecordControl.stop())
    message = status.get()
    while message.state != dai.RecordStatus.State.STOPPED:
        message = status.get()
    print(f"Stopped segment {message.segment} with {message.frameCount} frames")
    pipeline.stop()
