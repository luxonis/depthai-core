#!/usr/bin/env python3
"""Replay into a camera node. Requires a connected RVC4 device."""

import argparse

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("video", help="Video file or frame directory")
args = parser.parse_args()

with dai.Pipeline() as pipeline:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(args.video)
    replay.setFps(30)
    replay.setLoop(True)

    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, replay)
    output = camera.requestOutput((640, 480))
    queue = output.createOutputQueue()

    pipeline.start()
    while pipeline.isRunning():
        frame = queue.get()
        print(f"{frame.getWidth()}x{frame.getHeight()} sequence {frame.getSequenceNum()}")
        if frame.getSequenceNum() >= 30:
            break
    pipeline.stop()
