#!/usr/bin/env python3
"""Compare two rigs frame by frame - typically an estimated rig against the ground truth of a synthetic recording.

    python3 multi_device_rig_compare.py rec/rig.json rig_estimated.json

Every frame both rigs can reach from the reference is reported with its rotation error, its translation error and the
magnitude of the translation, because the inter-device magnitude is the part an image-only estimation cannot observe.
"""
import argparse
import math
from pathlib import Path

import numpy as np

import depthai as dai

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("reference", type=Path, help="Rig taken as the truth")
parser.add_argument("estimate", type=Path, help="Rig to compare against it")
parser.add_argument("--frame", help="Frame everything is expressed in, as deviceId:SOCKET. Defaults to the first frame of the reference rig")
args = parser.parse_args()

truth = dai.CalibrationHandler(str(args.reference))
estimate = dai.CalibrationHandler(str(args.estimate))

frames = sorted(
    {
        dai.CoordinateFrame(deviceId, socket)
        for deviceId, sockets in truth.getEepromData().devicesData.items()
        for socket, extrinsics in sockets.items()
    }
    | {
        extrinsics.getReferenceFrame()
        for sockets in truth.getEepromData().devicesData.values()
        for extrinsics in sockets.values()
    }
)
if args.frame:
    deviceId, socket = args.frame.split(":")
    anchor = dai.CoordinateFrame(deviceId, dai.CameraBoardSocket.__members__[socket])
else:
    anchor = frames[0]
print(f"expressed in {anchor}")

for frame in frames:
    if frame == anchor:
        continue
    try:
        expected = np.array(truth.getExtrinsics(frame.deviceId, frame.socket, anchor.deviceId, anchor.socket).getTransformationMatrix())
        actual = np.array(estimate.getExtrinsics(frame.deviceId, frame.socket, anchor.deviceId, anchor.socket).getTransformationMatrix())
    except RuntimeError:
        print(f"{frame}: no path to {anchor} in both rigs")
        continue
    difference = expected[:3, :3].T @ actual[:3, :3]
    angle = math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (np.trace(difference) - 1.0)))))
    expectedTranslation, actualTranslation = expected[:3, 3], actual[:3, 3]
    print(
        f"{frame}: rotation {angle:6.2f} deg, translation {np.linalg.norm(expectedTranslation - actualTranslation):6.1f} cm, "
        f"distance {np.linalg.norm(expectedTranslation):6.1f} -> {np.linalg.norm(actualTranslation):6.1f} cm"
    )
