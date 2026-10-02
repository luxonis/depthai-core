#!/usr/bin/env python3
"""Run ToF depth through VPP and StereoDepth on an RVC4 device. Press 'q' to quit.

  ┌──────┐      ┌───────────────┐                                       ┌─────┐
  | Left | ---> |               | ----------------left----------------> |     |
  └──────┘      |               |                                       |     | --left--> ┌────────┐
                | Rectification | ----------------right---------------> |     |           | Stereo | --depth->
  ┌───────┐     |               |                                       | Vpp |           |        |
  | Right | --> |               | --left--> ┌────────────┐              |     | --right-> └────────┘
  └───────┘     └───────────────┘           |            |              |     |
                                            | ImageAlign | ---depth---> |     |
  ┌─────┐                                   |            |              |     |
  | ToF | -------------depth--------------> └────────────┘              └─────┘
  └─────┘
"""

import argparse
import math

import cv2
import depthai as dai

FPS = 30


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--show-all", action="store_true",
                        help="stream ToF, aligned depth, and VPP images as well as fused depth; this can reduce FPS")
    parser.add_argument("--size", choices=("1280x800", "640x400"), default="1280x800",
                        help="stereo processing size (default: 1280x800)")
    parser.add_argument("--max-depth", type=float, default=5000,
                        help="fixed display depth limit in millimeters (default: 5000)")
    args = parser.parse_args()
    if not math.isfinite(args.max_depth) or args.max_depth <= 0:
        parser.error("--max-depth must be finite and positive")
    size = tuple(map(int, args.size.split("x")))
    print(f"DepthAI {dai.__version__}")
    with dai.Pipeline() as pipeline:
        if pipeline.getDefaultDevice().getPlatform() != dai.Platform.RVC4:
            raise RuntimeError("This script requires an RVC4 device")

        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        rect = pipeline.create(dai.node.Rectification)
        rect.setRunOnHost(False)
        rect.setOutputSize(*size)
        for camera, input_image in ((left, rect.input1), (right, rect.input2)):
            camera.requestOutput(size, type=dai.ImgFrame.Type.GRAY8,
                                 fps=FPS, enableUndistortion=False).link(input_image)

        tof = pipeline.create(dai.node.ToF).build(
            dai.CameraBoardSocket.AUTO, dai.ToFConfig.Profile.MID_RANGE, FPS)
        tof.setOutputUndistortion(False)

        align = pipeline.create(dai.node.ImageAlign)
        align.setRunOnHost(False)
        tof.tofBaseNode.depth.link(align.input)
        rect.output1.link(align.inputAlignTo)

        # Linking depth instead of disparity selects depth mode; confidence is optional.
        vpp = pipeline.create(dai.node.Vpp)
        rect.output1.link(vpp.left)
        rect.output2.link(vpp.right)
        align.outputAligned.link(vpp.depth)

        config = vpp.initialConfig
        config.blending = 0.6
        config.distanceGamma = 0.3
        config.maxPatchSize = 3
        config.patchColoringType = dai.VppConfig.PatchColoringType.RANDOM
        config.uniformPatch = False
        config.maxFPS = math.ceil(FPS)
        config.maxNumThreads = 1
        config.injectionParameters.useInjection = False
        config.injectionParameters.textureThreshold = 10.0

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_ACCURACY)
        stereo.setRectification(False)

        vpp.leftOut.link(stereo.left)
        vpp.rightOut.link(stereo.right)

        outputs = {"fused_depth": stereo.depth}
        if args.show_all:
            outputs.update({
                "tof_depth": tof.tofBaseNode.depth,
                "aligned_depth": align.outputAligned,
                "vpp_left": vpp.leftOut,
                "vpp_right": vpp.rightOut,
            })
        queues = {name: output.createOutputQueue(maxSize=2, blocking=False)
                  for name, output in outputs.items()}

        pipeline.start()
        while pipeline.isRunning():
            for name, queue in queues.items():
                frame = queue.tryGet()
                if frame is not None:
                    img = frame.getCvFrame()
                    if name in ("tof_depth", "aligned_depth", "fused_depth"):
                        # Fixed scale: outliers cannot change the brightness of the whole frame.
                        # Invalid zero depth stays black; depths beyond the limit saturate to white.
                        img = cv2.convertScaleAbs(img, alpha=255.0 / args.max_depth)
                    cv2.imshow(name, img)
            if cv2.waitKey(1) == ord("q"):
                break
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
