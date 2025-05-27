import depthai as dai
import argparse
import time
import signal
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default="test_video", help="Output file name (without extension)")

args = parser.parse_args()

with dai.Pipeline() as pipeline:
    def signal_handler(sig, frame):
        print("Interrupted, stopping the pipeline")
        pipeline.stop()
    signal.signal(signal.SIGINT, signal_handler)

    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    videoEncoder = pipeline.create(dai.node.VideoEncoder).build(cam.requestOutput((1280, 720), dai.ImgFrame.Type.NV12))
    videoEncoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)

    record = pipeline.create(dai.node.RecordVideo)
    record.setRecordVideoFile(Path(f"{args.output}.mp4"))

    videoEncoder.out.link(record.input)

    pipeline.start()
    print("Recording video. Press Ctrl+C to stop.")
    while pipeline.isRunning():
        time.sleep(1)

