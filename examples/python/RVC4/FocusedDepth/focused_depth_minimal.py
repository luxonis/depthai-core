#!/usr/bin/env python3
"""
Minimal RVC4 example: stereo cameras + static FocusedDepthRoi sent to FocusedDepth.

Requires OpenCV-enabled depthai (FocusedDepthCrop runs on host).
"""

import depthai as dai


def main() -> None:
    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        if device is None or device.getPlatform() != dai.Platform.RVC4:
            raise SystemExit("This example requires an RVC4 device.")

        pair = device.getStereoPairs()
        if not pair:
            raise SystemExit("No stereo pair on device.")

        left_cam = pipeline.create(dai.node.Camera).build(pair[0].left)
        right_cam = pipeline.create(dai.node.Camera).build(pair[0].right)
        size = (1280, 720)
        left_out = left_cam.requestOutput(size)
        right_out = right_cam.requestOutput(size)

        fd = pipeline.create(dai.node.FocusedDepth)
        fd.setBackend(dai.node.FocusedDepth.Backend.STEREO)
        left_out.link(fd.left)
        right_out.link(fd.right)

        roi_q = fd.roi.createInputQueue(maxSize=4, blocking=False)
        depth_q = fd.depth.createOutputQueue(maxSize=4, blocking=False)

        pipeline.start()

        frame_id = 0
        while pipeline.isRunning():
            roi = dai.FocusedDepthRoi()
            roi.roi = dai.Rect(400, 200, 480, 360, False)
            roi.sequenceHint = frame_id
            roi_q.send(roi)

            depth = depth_q.get()
            if depth is not None:
                print("Depth frame", depth.getSequenceNum(), depth.getWidth(), "x", depth.getHeight())
            frame_id += 1
            if frame_id > 30:
                break


if __name__ == "__main__":
    main()
