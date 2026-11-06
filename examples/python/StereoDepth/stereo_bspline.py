#!/usr/bin/env python3
"""RVC4 rectification from stored cubic spline coefficients, without a host mesh.

Requires the experimental matching SDK/firmware and factory calibration. This is
an SSH-friendly finite run. It never fits a model, evaluates a spline, loads a mesh,
or writes calibration. The device generates its warp once and reuses it.
"""
import argparse
from datetime import timedelta
import json
from pathlib import Path
import time

import numpy as np
import depthai as dai


def plane_statistics(depth, intrinsics, roi):
    """Untrimmed orthogonal fit on a deterministic sample; depth units are mm.

    The ROI must contain one textured physical plane. Coverage is over every ROI
    pixel; fitting uses at most about 25k pixels. This is a quick diagnostic, not
    an absolute accuracy or factory acceptance test.
    """
    height, width = depth.shape
    x0, y0, x1, y1 = (int(roi[0]*width), int(roi[1]*height),
                      int(roi[2]*width), int(roi[3]*height))
    region = depth[y0:y1, x0:x1]
    if region.size == 0:
        raise ValueError('ROI contains no pixels at this resolution')
    valid = np.isfinite(region) & (region > 0)
    result = {'roi': list(roi), 'valid_fraction': float(valid.mean())}
    step = max(1, int(np.ceil(np.sqrt(region.size/25000))))
    yy, xx = np.mgrid[y0:y1:step, x0:x1:step]
    z = np.asarray(depth[yy, xx], dtype=float)
    mask = np.isfinite(z) & (z > 0)
    fx, fy, cx, cy = intrinsics
    points = np.column_stack(((xx[mask]-cx)*z[mask]/fx,
                              (yy[mask]-cy)*z[mask]/fy, z[mask]))
    result['sample_points'] = len(points)
    if len(points) < 100:
        result['status'] = 'insufficient_valid_depth'
        return result
    center = points.mean(axis=0)
    eigenvalues, eigenvectors = np.linalg.eigh((points-center).T @ (points-center))
    if eigenvalues[1] <= 1e-6:
        result['status'] = 'degenerate_plane_support'
        return result
    residual = (points-center) @ eigenvectors[:, 0]
    result.update(status='measured', median_depth_mm=float(np.median(z[mask])),
                  plane_rms_mm=float(np.sqrt(np.mean(residual**2))),
                  plane_p95_mm=float(np.percentile(np.abs(residual), 95)))
    return result


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--device', required=True, help='Device IP or serial')
parser.add_argument('--model', choices=('LEGACY', 'BSPLINE'), default='BSPLINE',
                    help='Explicit rectification model; the SDK default remains LEGACY/AUTO')
parser.add_argument('--frames', type=int, default=300)
parser.add_argument('--check-only', action='store_true', help='Read stored model without starting cameras')
parser.add_argument('--stats-every', type=int, default=30)
parser.add_argument('--roi', type=float, nargs=4, default=(0.2, 0.2, 0.8, 0.8),
                    metavar=('X0', 'Y0', 'X1', 'Y1'), help='Normalized ROI containing one physical plane')
parser.add_argument('--output', type=Path, help='Save summary JSON and final depth NPY in this directory')
args = parser.parse_args()
if args.frames <= 0 or args.stats_every <= 0:
    parser.error('--frames and --stats-every must be positive')
if not (0 <= args.roi[0] < args.roi[2] <= 1 and 0 <= args.roi[1] < args.roi[3] <= 1):
    parser.error('--roi must satisfy 0 <= X0 < X1 <= 1 and 0 <= Y0 < Y1 <= 1')

with dai.Pipeline(dai.Device(dai.DeviceInfo(args.device))) as pipeline:
    handler = pipeline.getDefaultDevice().readCalibration()
    calibration = handler.getBsplineStereoCalibration()
    if args.model == 'BSPLINE' and calibration is None:
        raise RuntimeError('No B-spline stereo calibration stored on this device')
    if args.model == 'BSPLINE':
        calibration.validate()
        sockets = calibration.leftSocket, calibration.rightSocket
        size = calibration.width, calibration.height
        calibration_id = calibration.calibrationId
    else:
        legacy = handler.getEepromData().stereoRectificationData
        sockets = legacy.leftCameraSocket, legacy.rightCameraSocket
        size = (1280, 800)
        calibration_id = 'legacy'
    print(f'Selected {args.model}: {calibration_id}; {size}; {sockets}', flush=True)
    if args.check_only:
        raise SystemExit(0)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectificationModel(getattr(dai.node.StereoDepth.RectificationModel, args.model))
    stereo.setMeshStep(8, 8)
    stereo.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    stereo.setSubpixel(True)
    for socket, destination in zip(sockets, (stereo.left, stereo.right)):
        camera = pipeline.create(dai.node.Camera).build(socket)
        camera.requestOutput(size, dai.ImgFrame.Type.GRAY8, fps=30).link(destination)
    queue = stereo.depth.createOutputQueue(maxSize=4, blocking=False)
    pipeline.start()
    records = []
    first_timestamp = last_timestamp = None
    first_sequence = last_sequence = None
    gaps = 0
    started = time.monotonic()
    for i in range(args.frames):
        frame = queue.get(timedelta(seconds=30))
        if frame is None:
            raise RuntimeError('Timed out waiting for depth')
        timestamp = frame.getTimestamp().total_seconds()
        sequence = frame.getSequenceNum()
        if first_timestamp is None:
            first_timestamp, first_sequence = timestamp, sequence
        else:
            gaps += max(0, sequence-last_sequence-1)
        last_timestamp, last_sequence = timestamp, sequence
        if i % args.stats_every == 0 or i == args.frames-1:
            depth = frame.getFrame()
            k = np.asarray(frame.getTransformation().getIntrinsicMatrix())
            record = plane_statistics(depth, (k[0,0], k[1,1], k[0,2], k[1,2]), args.roi)
            record.update(frame=i+1, sequence=sequence)
            records.append(record)
            print(json.dumps(record), flush=True)
    elapsed = last_timestamp-first_timestamp
    summary = {'rectification_model': args.model, 'calibration_id': calibration_id, 'frames': args.frames,
               'device_timestamp_fps': (args.frames-1)/elapsed if elapsed > 0 else None,
               'sequence_gaps': gaps, 'host_elapsed_seconds': time.monotonic()-started,
               'plane_samples': records,
               'limitation': 'Quick untrimmed sampled ROI plane fit; no absolute accuracy or factory qualification.'}
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        np.save(args.output/'depth_mm.npy', depth)
    print(json.dumps(summary), flush=True)
