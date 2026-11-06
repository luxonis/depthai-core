#!/usr/bin/env python3
"""Replay into a host ImageManip node. No device is required."""

import argparse

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("video", help="Video file or frame directory")
args = parser.parse_args()

with dai.Pipeline(False) as pipeline:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(args.video)
    replay.setRunOnHost(True)
    replay.setOutFrameType(dai.ImgFrame.Type.NV12)
    replay.setFps(30)
    replay.setLoop(False)
    replay.setStopPipelineAtEnd(True)

    manip = pipeline.create(dai.node.ImageManip)
    manip.setRunOnHost()
    manip.setMaxOutputFrameSize(4 * 1024 * 1024)
    manip.initialConfig.setOutputSize(320, 240, dai.ImageManipConfig.ResizeMode.LETTERBOX)
    manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)

    replay.out.link(manip.inputImage)
    queue = manip.out.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        frame = queue.tryGet()
        if frame is None:
            continue
        print(f"{frame.getWidth()}x{frame.getHeight()} sequence {frame.getSequenceNum()}")
    pipeline.stop()
