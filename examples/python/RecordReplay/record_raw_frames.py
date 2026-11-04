import argparse
import time

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default=".", help="Directory that receives the recording")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    record = pipeline.create(dai.node.RecordVideo)
    record.setFormat(dai.RecordFormat.RAW_FRAMES)
    record.setOutputDirectory(args.output)
    record.setStreamName("frames")

    camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12).link(record.input)
    status = record.status.createOutputQueue(16, True)

    pipeline.start()
    print("Recording raw frames. Press Ctrl+C to stop.")
    while pipeline.isRunning():
        message = status.tryGet()
        if message is not None and message.state == dai.RecordStatus.State.STARTED:
            print(f"Recording {message.path}")
        time.sleep(0.2)
