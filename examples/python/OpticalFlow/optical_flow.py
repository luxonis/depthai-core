#!/usr/bin/env python3
"""Native RVC4 optical flow. Requires this host API and matching device firmware."""
import argparse
from pathlib import Path

import depthai as dai
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--device', help='Optional device ID or IP address')
parser.add_argument('--frames', type=int, default=30)
parser.add_argument('--output', type=Path, default=Path('optical-flow'))
args = parser.parse_args()
if not hasattr(dai.node, 'OpticalFlow') or not hasattr(dai.node.OpticalFlow, 'outputFlow'):
    raise RuntimeError('This depthai build does not expose OpticalFlow.outputFlow')
args.output.mkdir(parents=True, exist_ok=True)
device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
if device.getPlatform() != dai.Platform.RVC4:
    raise RuntimeError('Native optical-flow output is supported only on RVC4')
with device, dai.Pipeline(device) as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    flow_node = pipeline.create(dai.node.OpticalFlow)
    camera.requestOutput((640, 384), type=dai.ImgFrame.Type.GRAY8, fps=30).link(flow_node.inputImage)
    queue = flow_node.outputFlow.createOutputQueue(maxSize=2, blocking=True)
    pipeline.start()
    for index in range(args.frames):
        group = queue.get()
        flow, reference, target = group['flow'], group['reference'], group['target']
        forward = flow.getTensor('forward')
        backward = flow.getTensor('backward')
        confidence = flow.getTensor('confidence')[..., 0].astype(np.uint8)
        dt = (target.getTimestampDevice() - reference.getTimestampDevice()).total_seconds()
        if dt <= 0:
            raise RuntimeError('Non-positive frame-pair interval')
        np.savez(args.output / f'flow-{index:04d}.npz', forward=forward, backward=backward,
                 confidence=confidence, interval_s=dt,
                 reference_sequence=np.int64(reference.getSequenceNum()),
                 target_sequence=np.int64(target.getSequenceNum()))
        print(f'{reference.getSequenceNum()} -> {target.getSequenceNum()}: {dt*1000:.2f} ms, {forward.shape}')
