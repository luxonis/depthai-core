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
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(Path(args.inputVideo))
    replay.setOutFrameType(dai.ImgFrame.Type.NV12)
    replay.setLoop(False)

    imageManip = pipeline.create(dai.node.ImageManipV2)
    imageManip.initialConfig.setOutputSize(640, 400)
    imageManip.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
    replay.out.link(imageManip.inputImage)
    videoEncoder = pipeline.create(dai.node.VideoEncoder)
    imageManip.out.link(videoEncoder.input)
    videoEncoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H264_MAIN)
    outputEncoder = videoEncoder.out.createOutputQueue()
    manipOutQueue = imageManip.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning() and replay.isRunning():
        try:
            outFrame : dai.ImgFrame = manipOutQueue.get()
        except dai.MessageQueue.QueueException:
            # Replay stopped the pipeline
            break
        outFrameCv = outFrame.getCvFrame()
        print("Before trying to get frame")
        encodedFrame = outputEncoder.get()
        print("Got frame")
        cv2.imshow("video", outFrameCv)
        if cv2.waitKey(1) == ord('q'):
            print("Stopping pipeline")
            pipeline.stop()
            break
