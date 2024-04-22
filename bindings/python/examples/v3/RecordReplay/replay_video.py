import depthai as dai
import argparse
import time
import signal
from pathlib import Path
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--inputVideo", default="test_video.mp4", help="Input video name")

args = parser.parse_args()

# Check if the input video file exists
if not Path(args.inputVideo).exists():
    raise FileNotFoundError(f'Input video file not found: {args.inputVideo}')

with dai.Pipeline() as pipeline:
    def signal_handler(sig, frame):
        print("Interrupted, stopping the pipeline")
        pipeline.stop()
    signal.signal(signal.SIGINT, signal_handler)

    replay = pipeline.create(dai.node.Replay)
    replay.setReplayVideo(args.inputVideo)

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setResize(300, 300)
    replay.out.link(imageManip.inputImage)
    manipOutQueue = imageManip.out.createQueue()

    pipeline.start()
    while pipeline.isRunning():
        outFrame : dai.ImgFrame = manipOutQueue.get()
        frame = outFrame.getCvFrame()
        cv2.imshow("replay", frame)
        time.sleep(1)
