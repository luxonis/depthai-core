import argparse
import json
import signal
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import depthai as dai
import numpy as np

device = dai.Device()
FPS = 30
with dai.Pipeline(device) as pipeline:

    camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    camera_c_output = camera_c.requestOutput(
        (1280, 800), fps=FPS, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
    )
    camera_b_output = camera_b.requestOutput(
        (1280, 800), fps=FPS, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
    )

    camera_a_output = camera_a.requestOutput(
        (1280, 800),
        fps=FPS,
        enableUndistortion=False,
    )

    # stereo = pipeline.create(dai.node.StereoDepth).build(camera_b_output, camera_c_output, presetMode=dai.node.StereoDepth.PresetMode.FAST_DENSITY)
    # camera_a_output.link(stereo.inputAlignTo)
    
    cam_a_q = camera_a_output.createOutputQueue()
    cam_b_q = camera_b_output.createOutputQueue()
    cam_c_q = camera_c_output.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        cam_a_frame = cam_a_q.get()
        cam_b_frame = cam_b_q.get()
        cam_c_frame = cam_c_q.get()
        

        assert isinstance(cam_a_frame, dai.ImgFrame)
        assert isinstance(cam_b_frame, dai.ImgFrame)
        assert isinstance(cam_c_frame, dai.ImgFrame)

        cam_a_img = cam_a_frame.getCvFrame()
        cam_c_img = cam_c_frame.getCvFrame()
        cam_b_img = cam_b_frame.getCvFrame()

        cv2.imshow("cam_a", cam_a_img)
        cv2.imshow("cam_b", cam_b_img)
        cv2.imshow("cam_c", cam_c_img)
        
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
