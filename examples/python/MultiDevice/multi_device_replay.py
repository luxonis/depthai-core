#!/usr/bin/env python3
"""Replay a recorded multi-device session (multi_device_record.py) with no devices connected.

Two modes:
  stitch    (default) re-express every recorded stream in the reference frame and project it onto a plane
  calibrate estimate the rig from the recorded streams, using each device's stored calibration

Both read the per-device calibration and, for stitching, the rig json stored with the recording, so nothing has to be
read from a live device.
"""
import argparse
import json
from datetime import timedelta
from pathlib import Path

import cv2
import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("-i", "--input", type=Path, required=True, help="Recording directory written by multi_device_record.py")
parser.add_argument("-m", "--mode", choices=("stitch", "calibrate"), default="stitch")
parser.add_argument("--plane-point", type=float, nargs=3, help="A point of the plane, in cm in the reference camera frame (defaults to the plane stored with the recording)")
parser.add_argument("--plane-normal", type=float, nargs=3, help="Normal of the plane, in the reference camera frame (defaults to the plane stored with the recording)")
parser.add_argument("--range", type=float, default=600.0, help="How far from the cameras the plane is still rendered, in cm")
parser.add_argument("--view-size", type=int, nargs=2, default=(1400, 1400), help="Upper bound on the size of the computed view")
parser.add_argument("--samples", type=int, default=10, help="Image sets to accumulate before estimating the rig (calibrate mode)")
parser.add_argument("-o", "--output", type=Path, help="Save the first stitched frame / estimated rig here")
args = parser.parse_args()

manifest = json.loads((args.input / "session.json").read_text())
streams = manifest["streams"]

# The plane can be overridden on the command line, otherwise it comes from the recording (else a level floor 1.5 m down)
storedPlane = manifest.get("plane", {})
planePoint = args.plane_point or storedPlane.get("point", (0.0, 150.0, 0.0))
planeNormal = args.plane_normal or storedPlane.get("normal", (0.0, -1.0, 0.0))
reference = dai.CoordinateFrame(manifest["reference"]["deviceId"], dai.CameraBoardSocket.__members__[manifest["reference"]["socket"]])
anchorSocket = dai.CameraBoardSocket.__members__[manifest["reference"]["socket"]]
calibrations = {deviceId: dai.CalibrationHandler(str(args.input / f"{deviceId}_calibration.json")) for deviceId in manifest["devices"]}


def frameOf(stream):
    return dai.CoordinateFrame(stream["deviceId"], dai.CameraBoardSocket.__members__[stream["socket"]])


def makeReplay(pipeline, stream):
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(args.input / stream["video"])
    replay.setReplayMetadataFile(args.input / stream["metadata"])
    replay.setOutFrameType(dai.ImgFrame.Type.NV12)
    return replay


def augmentedRig():
    """The stored rig holds only inter-device edges; add each device's intra-device edges so a device-less pipeline
    can transform any camera to the reference (the same completion CoordinateFrameTransform does from live devices)."""
    rig = dai.MultiDeviceCalibrationHandler(str(args.input / "rig.json"))
    candidates = (dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_D)
    for deviceId, calibration in calibrations.items():
        anchor = dai.CoordinateFrame(deviceId, anchorSocket)
        for socket in candidates:
            frame = dai.CoordinateFrame(deviceId, socket)
            if frame == anchor or rig.canTransform(frame, anchor):
                continue
            try:
                matrix = calibration.getCameraExtrinsics(socket, anchorSocket)
            except Exception:
                continue
            edge = dai.RigEdge()
            edge.from_ = frame
            edge.to = anchor
            edge.transform = dai.Extrinsics()
            edge.transform.setTransformationMatrix(matrix)
            edge.transform.setReferenceFrame(anchor)
            edge.source = "device-calibration"
            rig.setEdge(edge)
    return rig


with dai.Pipeline(createImplicitDevice=False) as pipeline:
    replays = [makeReplay(pipeline, stream) for stream in streams]

    if args.mode == "stitch":
        unified = pipeline.create(dai.node.CoordinateFrameTransform).build([replay.out for replay in replays], reference)
        unified.setCalibration(augmentedRig())
        for index, stream in enumerate(streams):
            unified.setSourceFrame(index, frameOf(stream))

        stitching = pipeline.create(dai.node.Stitching).build([unified.outputs[f"output{index}"] for index in range(len(replays))])
        stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
        stitching.setPlane(dai.Point3f(*planePoint), dai.Point3f(*planeNormal))
        stitching.setMaxRange(args.range)
        stitching.setMaxViewSize(*args.view_size)
        # Independent devices are not hardware-synced, so replayed streams only need to be grouped loosely
        stitching.setSyncThreshold(timedelta(seconds=max(1.0, 5.0 / manifest["fps"])))
        projectedQueue = stitching.out.createOutputQueue()

        pipeline.start()
        print(f"Projecting {stitching.getNumInputs()} recorded cameras onto the plane, press q to exit", flush=True)
        while pipeline.isRunning():
            projected = projectedQueue.get().getCvFrame()
            if args.output is not None:
                cv2.imwrite(str(args.output), projected)
                print(f"saved {projected.shape[1]}x{projected.shape[0]} to {args.output}", flush=True)
                break
            cv2.imshow("planar stitch", projected)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    else:
        sources = [(frameOf(stream), replay.out) for stream, replay in zip(streams, replays)]
        calibration = pipeline.create(dai.node.MultiDeviceCalibration).build(sources)
        for deviceId, handler in calibrations.items():
            calibration.setDeviceCalibration(deviceId, handler)
        calibration.setSampleCount(args.samples)
        # Independent devices are not hardware-synced, so replayed streams only need to be grouped loosely
        calibration.sync.setSyncThreshold(timedelta(seconds=max(1.0, 5.0 / manifest["fps"])))

        # Seed the optimization with the stored rig, if any
        if manifest.get("hasRig"):
            rig = dai.MultiDeviceCalibrationHandler(str(args.input / "rig.json"))
            for deviceId in manifest["devices"][1:]:
                source = dai.CoordinateFrame(deviceId, anchorSocket)
                if not rig.canTransform(source, reference):
                    continue
                guess = dai.Extrinsics()
                guess.setTransformationMatrix(rig.getTransform(source, reference))
                calibration.setInitialGuess(source, reference, guess)

        rigQueue = calibration.rigCalibration.createOutputQueue()
        pipeline.start()
        print(f"Estimating the rig from {len(streams)} recorded streams", flush=True)
        result = rigQueue.get()
        print(f"passed={result.passed} confidence={result.dataConfidence:.3f} info={result.info!r}", flush=True)
        if result.passed and args.output is not None:
            dai.MultiDeviceCalibrationHandler(result.calibration).toJsonFile(str(args.output))
            print(f"rig written to {args.output}", flush=True)
