#!/usr/bin/env python3
"""
RGB object detection fused with ``dai.node.Depth`` for metric distance on the depth image.

Pipeline: ``DetectionNetwork`` on the color camera (when present), ``Depth`` for depth, and
``SpatialLocationCalculator`` for 3D positions. Distances are drawn on the colorized depth view
(fixed 0..15 m range).

**RVC4 / others:** depth is aligned to RGB with on-device ``ImageAlign``.

**RVC2:** uses ``StereoDepth.inputAlignTo`` (no ``ImageAlign``) to stay within device ImageManip
limits. On boards where ``AUTO`` would select ToF (e.g. OAK-D ToF), this example forces
``Depth.Algorithm.STEREO`` instead — ToF depth plus RGB detection cannot be aligned reliably on
RVC2 without exhausting ImageManip memory.

Requires a color/RGB camera in addition to the device stereo pair.
"""

import sys
from pathlib import Path
from typing import NamedTuple

import cv2
import depthai as dai

sys.path.insert(0, str(Path(__file__).resolve().parent))
import depth_example_common as dec


class DetectionProfile(NamedTuple):
    model: dai.NNModelDescription
    nn_size: tuple[int, int]
    nn_fps: float
    undistort: bool
    label: str


def detection_profile(device: dai.Device) -> DetectionProfile:
    platform = device.getPlatform()
    if platform == dai.Platform.RVC2:
        return DetectionProfile(
            model=dai.NNModelDescription("mobilenet-ssd", platform="RVC2"),
            nn_size=(300, 300),
            nn_fps=10.0,
            undistort=False,
            label="mobilenet-ssd@300x300 (RVC2)",
        )
    return DetectionProfile(
        model=dai.NNModelDescription("yolov6-nano"),
        nn_size=(640, 400),
        nn_fps=20.0,
        undistort=True,
        label="yolov6-nano@640x400",
    )


class DepthSpatialVisualizer(dai.node.HostNode):
    def __init__(self) -> None:
        dai.node.HostNode.__init__(self)
        self.sendProcessingToPipeline(True)

    def build(self, depth: dai.Node.Output, spatial_detections: dai.Node.Output):
        # Parameter annotations must be real types (not postponed); link_args introspects them.
        self.link_args(depth, spatial_detections)

    def process(self, depth_msg, spatial_msg):
        depth_color = dec.colorizeDepthMm(depth_msg.getFrame())
        h, w = depth_color.shape[:2]

        for detection in spatial_msg.detections:
            roi_data = detection.boundingBoxMapping
            roi = roi_data.roi.denormalize(w, h)
            top_left = roi.topLeft()
            bottom_right = roi.bottomRight()
            x1, y1 = int(top_left.x), int(top_left.y)
            x2, y2 = int(bottom_right.x), int(bottom_right.y)
            cv2.rectangle(depth_color, (x1, y1), (x2, y2), (255, 255, 255), 2)

            z_mm = int(detection.spatialCoordinates.z)
            label = detection.labelName
            cv2.putText(
                depth_color,
                f"{label} {z_mm} mm",
                (x1 + 4, max(y1 + 18, 18)),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        cv2.imshow("depth + detections", depth_color)
        if cv2.waitKey(1) == ord("q"):
            self.stopPipeline()


with dai.Pipeline() as pipeline:
    device = pipeline.getDefaultDevice()
    if device is None:
        print("Connect a device.", file=sys.stderr)
        sys.exit(1)

    platform = device.getPlatform()

    try:
        stereo_pair = dec.require_first_stereo_pair(device)
    except RuntimeError as ex:
        print(ex, file=sys.stderr)
        sys.exit(1)

    rgb_socket = dec.find_color_camera_socket(device, stereo_pair)
    if rgb_socket is None:
        print(
            "No color/RGB camera found (mono-only device). "
            "This example needs an RGB camera for detection.",
            file=sys.stderr,
        )
        sys.exit(1)

    profile = detection_profile(device)

    nn_cap = dai.ImgFrameCapability()
    nn_cap.size.fixed(profile.nn_size)
    nn_cap.fps.fixed(profile.nn_fps)
    nn_cap.enableUndistortion = profile.undistort

    cam_rgb = pipeline.create(dai.node.Camera).build(rgb_socket, sensorFps=profile.nn_fps)
    detection_network = pipeline.create(dai.node.DetectionNetwork).build(cam_rgb, profile.model, nn_cap)

    depth_node = pipeline.create(dai.node.Depth)
    spatial_calc = pipeline.create(dai.node.SpatialLocationCalculator)
    depth_align_mode = ""
    use_stereo_depth = False

    if platform == dai.Platform.RVC2:
        if dec.device_has_tof_sensor(device):
            depth_node.build(dai.node.Depth.Algorithm.STEREO)
            use_stereo_depth = True
        depth_node.depth  # lazy-wire (AUTO → StereoDepth when no ToF sensor)
        stereo_backend = dec.find_stereo_depth_in_pipeline(pipeline)
        if stereo_backend is None:
            print(
                "RVC2: Depth did not create a StereoDepth backend. "
                "This example does not support AUTO→ToF with RGB spatial detection.",
                file=sys.stderr,
            )
            sys.exit(1)
        detection_network.passthrough.link(stereo_backend.inputAlignTo)
        stereo_backend.depth.link(spatial_calc.inputDepth)
        depth_align_mode = "StereoDepth.inputAlignTo"
    else:
        align = pipeline.create(dai.node.ImageAlign)
        depth_node.depth.link(align.input)
        detection_network.passthrough.link(align.inputAlignTo)
        align.outputAligned.link(spatial_calc.inputDepth)
        depth_align_mode = "ImageAlign (device)"

    detection_network.out.link(spatial_calc.inputDetections)
    detection_network.input.setBlocking(False)
    spatial_calc.initialConfig.setSegmentationPassthrough(False)

    visualizer = pipeline.create(DepthSpatialVisualizer)
    visualizer.build(spatial_calc.passthroughDepth, spatial_calc.outputDetections)

    depth_mode = "STEREO (ToF board)" if use_stereo_depth else "AUTO"
    print(
        f"Depth ({depth_mode}) + detection on {rgb_socket}, "
        f"profile: {profile.label}, align: {depth_align_mode}",
        flush=True,
    )
    pipeline.run()
