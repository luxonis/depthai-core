#!/usr/bin/env python3
"""Bird's eye view stitching of a virtual multi-camera rig.

Renders a synthetic dataset - a textured ground plane seen by several cameras mounted at a known height and pitched
down - and feeds it to the Stitching node in PLANAR_PROJECTION mode. No device is needed, so it doubles as a way to
play with the plane and the view without a rig on the desk.
"""
import argparse
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(640, 400), help="Resolution of every virtual camera")
parser.add_argument("-f", "--focal", type=float, default=380.0, help="Focal length of every virtual camera, in pixels")
parser.add_argument("--height", type=float, default=100.0, help="Height the cameras are mounted at, in cm")
parser.add_argument("--pitch", type=float, default=35.0, help="How far below the horizon the cameras look, in degrees")
parser.add_argument("--yaw", type=float, nargs="+", default=[-35.0, 0.0, 35.0], help="Heading of every camera, in degrees")
parser.add_argument("--baseline", type=float, default=140.0, help="Distance between neighbouring cameras of the rig, in cm")
parser.add_argument("--range", type=float, default=400.0, help="How far from a camera the ground is still painted, in cm")
parser.add_argument("--view-size", type=int, nargs=2, default=(960, 960), help="Upper bound on the size of the computed view")
parser.add_argument("--top-down", action="store_true", help="Render from a camera hanging above the rig instead of the computed view")
parser.add_argument("-o", "--output", type=Path, help="Directory to write the dataset and the result to")
args = parser.parse_args()

WIDTH, HEIGHT = args.resolution
TEXTURE_SIZE = 1600  # pixels
TEXTURE_SCALE = 0.5  # cm per texture pixel
REFERENCE_FRAME = dai.CoordinateFrame("virtualrig", dai.CameraBoardSocket.CAM_A)


def render_ground():
    """Ground texture: blocks of random color plus a metric grid, so that every part of the plane looks different."""
    rng = np.random.default_rng(20240517)
    blocks = rng.integers(0, 200, size=(TEXTURE_SIZE // 20, TEXTURE_SIZE // 20, 3), dtype=np.uint8)
    texture = np.repeat(np.repeat(blocks, 20, axis=0), 20, axis=1)

    for meters in range(-4, 5):
        offset = int(TEXTURE_SIZE / 2 + meters * 100 / TEXTURE_SCALE)
        if 0 <= offset < TEXTURE_SIZE:
            texture[offset - 1 : offset + 2, :] = (255, 255, 255)
            texture[:, offset - 1 : offset + 2] = (255, 255, 255)
    return np.ascontiguousarray(texture)


def make_camera(x, y, yaw_degrees):
    """Pose of a camera mounted at (x, y) in the world, in the depthai convention: x right, y down, z forward."""
    yaw, pitch = np.deg2rad(yaw_degrees), np.deg2rad(args.pitch)
    forward = np.array([np.sin(yaw) * np.cos(pitch), np.cos(yaw) * np.cos(pitch), -np.sin(pitch)])
    right = np.array([np.cos(yaw), -np.sin(yaw), 0.0])
    down = np.cross(forward, right)

    rotation = np.column_stack([right, down, forward])
    center = np.array([x, y, args.height])
    intrinsics = np.array([[args.focal, 0.0, (WIDTH - 1) / 2], [0.0, args.focal, (HEIGHT - 1) / 2], [0.0, 0.0, 1.0]])
    return rotation, center, intrinsics


def render_view(texture, camera):
    """What the camera sees of the ground, the ground being the z == 0 plane the texture is painted on."""
    rotation, center, intrinsics = camera
    world_to_camera = rotation.T
    origin = np.array([-0.5 * TEXTURE_SIZE * TEXTURE_SCALE, -0.5 * TEXTURE_SIZE * TEXTURE_SCALE, 0.0])
    columns = np.column_stack(
        [
            world_to_camera @ np.array([TEXTURE_SCALE, 0.0, 0.0]),
            world_to_camera @ np.array([0.0, TEXTURE_SCALE, 0.0]),
            world_to_camera @ (origin - center),
        ]
    )
    return cv2.warpPerspective(texture, intrinsics @ columns, (WIDTH, HEIGHT), flags=cv2.INTER_LINEAR)


def to_frame(image, rotation, translation, intrinsics, sequence_num):
    """An ImgFrame carrying the calibration a real pipeline would have re-expressed with CoordinateFrameTransform."""
    extrinsics = dai.Extrinsics()
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = translation
    extrinsics.setTransformationMatrix(pose.tolist(), dai.LengthUnit.CENTIMETER)
    extrinsics.setReferenceFrame(REFERENCE_FRAME)

    frame = dai.ImgFrame()
    frame.setCvFrame(image, dai.ImgFrame.Type.BGR888i)
    frame.setSequenceNum(sequence_num)
    frame.setTransformation(dai.ImgTransformation(WIDTH, HEIGHT, intrinsics.tolist(), dai.CameraModel.Perspective, [], extrinsics))
    return frame


texture = render_ground()
# The rig: cameras evenly spread along the x axis, each looking in its own direction
cameras = [make_camera(args.baseline * (index - 0.5 * (len(args.yaw) - 1)), -50.0, yaw) for index, yaw in enumerate(args.yaw)]
images = [render_view(texture, camera) for camera in cameras]

# Everything is expressed in the frame of the first camera, which is what CoordinateFrameTransform would produce
reference_rotation = cameras[0][0].T
reference_translation = -reference_rotation @ cameras[0][1]


def to_reference(point):
    return reference_rotation @ np.asarray(point, dtype=float) + reference_translation


with dai.Pipeline(createImplicitDevice=False) as pipeline:
    stitching = pipeline.create(dai.node.Stitching).build(len(cameras))
    stitching.setMode(dai.node.Stitching.Mode.PLANAR_PROJECTION)
    # The ground, i.e. the z == 0 plane of the world, expressed in the frame of the reference camera
    stitching.setPlane(dai.Point3f(*to_reference([0, 0, 0])), dai.Point3f(*(reference_rotation @ [0, 0, 1])))
    stitching.setMaxRange(args.range)
    stitching.setMaxViewSize(*args.view_size)
    if args.top_down:
        stitching.setView(
            dai.node.Stitching.VirtualCamera.lookAt(
                position=dai.Point3f(*to_reference([0, 100, 400])),
                target=dai.Point3f(*to_reference([0, 100, 0])),
                up=dai.Point3f(*(reference_rotation @ [0, 1, 0])),
                hFovDegrees=90.0,
                width=args.view_size[0],
                height=args.view_size[1],
            )
        )

    inputQueues = [stitching.inputs[f"input{index}"].createInputQueue() for index in range(len(cameras))]
    projectedQueue = stitching.out.createOutputQueue()

    pipeline.start()
    for index, (image, camera) in enumerate(zip(images, cameras)):
        rotation = reference_rotation @ camera[0]
        translation = reference_rotation @ camera[1] + reference_translation
        inputQueues[index].send(to_frame(image, rotation, translation, camera[2], 0))

    projected = projectedQueue.get()
    pipeline.stop()

result = projected.getCvFrame()
print(f"Rendered {result.shape[1]}x{result.shape[0]} pixels of the plane from {len(cameras)} virtual cameras")

if args.output:
    # Writing the dataset out is what a headless run is for, so it does not open any window
    args.output.mkdir(parents=True, exist_ok=True)
    for index, image in enumerate(images):
        cv2.imwrite(str(args.output / f"camera{index}.png"), image)
    cv2.imwrite(str(args.output / "ground_truth.png"), texture)
    cv2.imwrite(str(args.output / "planar_stitch.png"), result)
    print(f"Dataset written to {args.output}")
else:
    cv2.imshow("cameras", np.hstack(images))
    cv2.imshow("planar stitch", result)
    print("Press any key to exit")
    cv2.waitKey(0)
