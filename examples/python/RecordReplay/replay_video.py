import depthai as dai
import argparse
import time
from pathlib import Path
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--inputVideo", default="test_video.mp4", help="Input video name")
parser.add_argument("-m", "--inputMetadata", default="test_video.mcap", help="Input metadata name")

args = parser.parse_args()

# Check if the input video file exists
if not Path(args.inputVideo).exists():
    print("First record a video using the record_video.py script")
    raise FileNotFoundError(f'Input video file not found: {args.inputVideo}')

with dai.Pipeline() as pipeline:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(Path(args.inputVideo))
    if Path(args.inputMetadata).exists():
        replay.setReplayMetadataFile(Path(args.inputMetadata))
    replay.setOutFrameType(dai.ImgFrame.Type.NV12)
    replay.setLoop(False)

    videoOut = replay.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning() and replay.isRunning():
        try:
            outFrame : dai.ImgFrame = videoOut.get()
        except dai.MessageQueue.QueueException:
            # Replay stopped the pipeline
            break
        outFrameCv = outFrame.getCvFrame()
        cv2.imshow("video", outFrameCv)
        if cv2.waitKey(1) == ord('q'):
            print("Stopping pipeline")
            pipeline.stop()
            break
