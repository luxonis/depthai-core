#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import sys


def exit_usage():
    print(
        "WRONG USAGE! correct usage example:\n"
        "python timings.py 640 480 0 30\n"
        "where 0 is resize mode: 0 == CROP, 1 == STRETCH, 2 == LETTERBOX\n"
        "and 30 is FPS"
    )
    exit(1)


args = sys.argv[1:]
if len(args) != 4:
    exit_usage()

with dai.Pipeline() as pipeline:
    script = pipeline.create(dai.node.Script)
    script.setScript(
        """
    while True:
        frame = node.inputs['frames'].get()
        node.outputs['out'].send(Buffer(32))
"""
    )
    cameraNode = pipeline.create(dai.node.Camera)
    cameraNode.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cap = dai.ImgFrameCapability()
    cap.type = dai.ImgFrame.Type.BGR888i
    cropArg = int(args[2])
    if cropArg == 0:
        cap.resizeMode = dai.ImgResizeMode.CROP
    elif cropArg == 1:
        cap.resizeMode = dai.ImgResizeMode.STRETCH
    elif cropArg == 2:
        cap.resizeMode = dai.ImgResizeMode.LETTERBOX
    else:
        exit_usage()
    cap.size.fixed([int(args[0]), int(args[1])])
    cap.fps.fixed(int(args[3]))
    cameraNode.requestOutput(cap, False).link(script.inputs["frames"])
    out = script.outputs["out"].createQueue()
    pipeline.start()
    nextSecondStart = time.time_ns() + 10**9
    fpsCount = 0
    while pipeline.isRunning():
        out.get()
        if time.time_ns() > nextSecondStart:
            print(f"FPS: {fpsCount}")
            nextSecondStart += 10**9
            fpsCount = 0
        fpsCount += 1
