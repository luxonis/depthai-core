#!/usr/bin/env python3
"""Minimal ToF script showing the main output stream.

Displays depth.
For more streams, see tof_all_queues.py.
On RVC4, select a VD55H1 mode with --sensor-mode FREQUENCY_2_BINNED.

Press 'q' to quit.
"""

import argparse

import cv2
import depthai as dai

FPS = 30.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sensor-mode",
        choices=list(dai.node.ToF.SensorMode.__members__),
        default="FREQUENCY_3",
        help="VD55H1 startup mode (RVC4 only; leave the default on RVC2)",
    )
    args = parser.parse_args()

    pipeline = dai.Pipeline()

    minDepth = 100.0
    maxDepth = 7000.0

    profile = dai.ToFConfig.Profile.MID_RANGE

    tof = pipeline.create(dai.node.ToF).build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        profile=profile,
        fps=FPS,
        sensorMode=dai.node.ToF.SensorMode.__members__[args.sensor_mode],
    )

    depthOutputQueue = tof.depth.createOutputQueue()

    with pipeline as p:
        p.start()
        while p.isRunning():
            depth = depthOutputQueue.get()
            cv2.imshow("depth", dai.utility.colorizeDepthFrame(depth, minDepth, maxDepth, useLog=True).getCvFrame())

            if cv2.waitKey(1) == ord("q"):
                break


if __name__ == "__main__":
    main()
