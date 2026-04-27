#!/usr/bin/env python3

from datetime import timedelta

import cv2
import depthai as dai
import numpy as np

FIRST_SOCKET = dai.CameraBoardSocket.CAM_A
SECOND_SOCKET = dai.CameraBoardSocket.CAM_D
PRESET_MODE = dai.ImageFiltersPresetMode.TOF_HIGH_RANGE
SYNC_THRESHOLD = timedelta(milliseconds=2)

deviceConfig = dai.Device.Config()
deviceConfig.board.ffcFsyncGeneratorGpio = 40  # OAK-FFC-4P FSIN/FSYNC generator GPIO
deviceConfig.board.ffcFsyncGeneratorFps = 30
device = dai.Device(deviceConfig)


def colorizeDepth(frameDepth: np.ndarray) -> np.ndarray:
    invalidMask = frameDepth == 0  # zero depth is invalid

    try:
        minDepth = np.percentile(frameDepth[frameDepth != 0], 3)
        maxDepth = np.percentile(frameDepth[frameDepth != 0], 95)
        logDepth = np.log(frameDepth, where=frameDepth != 0)
        logMinDepth = np.log(minDepth)
        logMaxDepth = np.log(maxDepth)
        np.nan_to_num(logDepth, copy=False, nan=logMinDepth)
        logDepth = np.clip(logDepth, logMinDepth, logMaxDepth)
        depthFrameColor = np.interp(logDepth, (logMinDepth, logMaxDepth), (0, 255))
        depthFrameColor = np.nan_to_num(depthFrameColor)
        depthFrameColor = depthFrameColor.astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        depthFrameColor[invalidMask] = 0
    except IndexError:
        depthFrameColor = np.zeros(
            (frameDepth.shape[0], frameDepth.shape[1], 3), dtype=np.uint8
        )
    except Exception as e:
        raise e
    return depthFrameColor


def main():
    pipeline = dai.Pipeline(device)

    first_tof = pipeline.create(dai.node.ToF).build(FIRST_SOCKET, PRESET_MODE)
    second_tof = pipeline.create(dai.node.ToF).build(SECOND_SOCKET, PRESET_MODE)
    first_tof.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    first_tof.initialControl.setFrameSyncId(0)
    second_tof.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    second_tof.initialControl.setFrameSyncId(1)

    sync = pipeline.create(dai.node.Sync)
    sync.setRunOnHost(True)
    sync.setSyncThreshold(SYNC_THRESHOLD)
    first_tof.depth.link(sync.inputs["cam_a_depth"])
    second_tof.depth.link(sync.inputs["cam_d_depth"])
    output_queue = sync.out.createOutputQueue()

    with pipeline as p:
        p.start()
        while p.isRunning():
            message_group: dai.MessageGroup = output_queue.get()
            cam_a_depth: dai.ImgFrame = message_group["cam_a_depth"]
            cam_d_depth: dai.ImgFrame = message_group["cam_d_depth"]

            cv2.imshow("CAM_A depth", colorizeDepth(cam_a_depth.getFrame()))
            cv2.imshow("CAM_D depth", colorizeDepth(cam_d_depth.getFrame()))

            cam_a_ts = cam_a_depth.getTimestampDevice()
            cam_d_ts = cam_d_depth.getTimestampDevice()
            print(
                f"cam_a_ts={cam_a_ts} cam_d_ts={cam_d_ts} "
                f"delta_ms={(cam_d_ts - cam_a_ts).total_seconds() * 1000:.3f}"
            )

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
