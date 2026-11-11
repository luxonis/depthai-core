#!/usr/bin/env python3
"""Rectify a calibrated perspective stereo pair (matching SDK/firmware required).

Try --alpha 0 for cropping, --alpha 1 for more source pixels, or a negative
value for extra cropping. Alpha is configured before starting the pipeline.
Both platforms use grayscale. On RVC4, try --backend GPU or --backend HW for EVA.
"""

import argparse
import math

import cv2
import depthai as dai


def stereoView(left, right, guides=False):
    images = [left.getCvFrame(), right.getCvFrame()]
    images = [cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image for image in images]
    view = cv2.hconcat(images)
    if guides:
        for y in range(40, view.shape[0], 40):
            cv2.line(view, (0, y), (view.shape[1] - 1, y), (0, 255, 0), 1)
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--alpha", type=float, default=0.0, help="0: crop, 1: retain more pixels; negative: crop further (default: 0)")
    parser.add_argument("--backend", choices=["AUTO", "CPU", "GPU", "HW"], default="AUTO", help="Rectification backend (default: AUTO)")
    args = parser.parse_args()
    if not math.isfinite(args.alpha):
        parser.error("--alpha must be finite")

    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        isRvc2 = device.getPlatform() == dai.Platform.RVC2
        if isRvc2 and args.backend not in ("AUTO", "HW"):
            raise RuntimeError("RVC2 supports AUTO and HW backends only")
        pairs = device.getAvailableStereoPairs()
        if not pairs:
            raise RuntimeError("This example requires a calibrated stereo camera pair")
        pair = pairs[0]
        left = pipeline.create(dai.node.Camera).build(pair.left)
        right = pipeline.create(dai.node.Camera).build(pair.right)
        frameType = dai.ImgFrame.Type.GRAY8

        rectification = pipeline.create(dai.node.Rectification)
        rectification.setBackend(getattr(dai.node.Rectification.Backend, args.backend))
        rectification.setAlphaScaling(args.alpha)
        # Undistortion, rectification and alpha scaling are combined into one map per camera.
        left.requestOutput((640, 400), frameType, dai.ImgResizeMode.CROP, 15.0, False).link(rectification.input1)
        right.requestOutput((640, 400), frameType, dai.ImgResizeMode.CROP, 15.0, False).link(rectification.input2)

        # Keep display queues small and nonblocking so previews cannot exhaust device frame pools.
        originalLeft = rectification.passthrough1.createOutputQueue(1, False)
        originalRight = rectification.passthrough2.createOutputQueue(1, False)
        rectifiedLeft = rectification.output1.createOutputQueue(1, False)
        rectifiedRight = rectification.output2.createOutputQueue(1, False)
        title = f"Rectified: alpha={args.alpha:g}, backend={args.backend}"
        print(f"Stereo pair: {pair.left}, {pair.right}. Press q to quit.")
        pipeline.start()
        while pipeline.isRunning():
            cv2.imshow("Original stereo", stereoView(originalLeft.get(), originalRight.get()))
            cv2.imshow(title, stereoView(rectifiedLeft.get(), rectifiedRight.get(), guides=True))
            if cv2.waitKey(1) == ord("q"):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
