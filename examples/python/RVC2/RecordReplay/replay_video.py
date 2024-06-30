import depthai as dai
import argparse
import time
from pathlib import Path
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--inputVideo", default="test_video.mp4", help="Input video name")

args = parser.parse_args()

# Check if the input video file exists
if not Path(args.inputVideo).exists():
    print("First record a video using the record_video.py script")
    raise FileNotFoundError(f'Input video file not found: {args.inputVideo}')

with dai.Pipeline() as pipeline:
    replay = pipeline.create(dai.node.Replay)
    replay.setReplayVideo(args.inputVideo)
    replay.setOutFrameType(dai.ImgFrame.Type.BGR888p)

    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setResize(300, 300)
    imageManip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    replay.out.link(imageManip.inputImage)
    manipOutQueue = imageManip.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning() and replay.isRunning():
        outFrame : dai.ImgFrame = manipOutQueue.get()
        outFrameCv = outFrame.getCvFrame()
        cv2.imshow("video", outFrameCv)
        if cv2.waitKey(1) == ord('q'):
            print("Stopping pipeline")
            pipeline.stop()
            break
