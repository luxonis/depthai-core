#!/usr/bin/env python3
"""RVC4 on-device local motion deblurring. No recordings or external assets."""
import argparse
from datetime import timedelta
import depthai as dai
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--device', help='Optional device ID or address')
parser.add_argument('--frames', type=int, default=30)
parser.add_argument('--cpu', action='store_true', help='Use the on-device CPU reference FIR')
parser.add_argument('--show', action='store_true', help='Display original/restored side by side (requires OpenCV)')
args = parser.parse_args()
if args.show:
    import cv2

device = dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device()
with device, dai.Pipeline(device) as pipeline:
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    flow = pipeline.create(dai.node.OpticalFlow)
    deblur = pipeline.create(dai.node.Deblur)
    config = deblur.getConfig()
    config.useOpenCL = not args.cpu
    # Uses the actual frame exposure metadata by default. Set exposureTimeUs
    # only when exposure metadata is unavailable and the true exposure is known.
    deblur.setConfig(config)
    camera.requestOutput((640, 384), type=dai.ImgFrame.Type.GRAY8, fps=15).link(flow.inputImage)
    flow.passthroughInputImage.link(deblur.inputImage)
    flow.outputFlow.link(deblur.inputFlow)
    restored = deblur.output.createOutputQueue(maxSize=2, blocking=True)
    original = deblur.passthroughInputImage.createOutputQueue(maxSize=2, blocking=True)
    masks = deblur.outputMask.createOutputQueue(maxSize=2, blocking=True)
    pipeline.start()
    for _ in range(args.frames):
        out, src, mask = (queue.get(timedelta(seconds=15)) for queue in (restored, original, masks))
        if out is None or src is None or mask is None:
            raise RuntimeError('Timed out waiting for Deblur outputs')
        assert out.getSequenceNum() == src.getSequenceNum() == mask.getSequenceNum()
        print(f'frame {out.getSequenceNum()}: accepted motion {(mask.getFrame() > 0).mean():.1%}, '
              f'exposure {src.getExposureTime().total_seconds()*1000:.2f} ms')
        if args.show:
            cv2.imshow('Original | Deblur', np.hstack((src.getFrame(), out.getFrame())))
            if cv2.waitKey(1) == 27:
                break
if args.show:
    cv2.destroyAllWindows()
