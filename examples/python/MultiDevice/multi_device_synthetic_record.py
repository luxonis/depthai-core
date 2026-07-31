#!/usr/bin/env python3
"""Render a synthetic multi-device recording that replays like a real one (multi_device_record.py).

Three virtual devices with a stereo pair each look at a textured floor from different angles, so the recording
exercises the whole multi-device path - CoordinateFrameTransform, planar Stitching and MultiDeviceCalibration - with
geometry that is known exactly. It writes the layout multi_device_replay.py expects:

    <deviceId>_<SOCKET>.avi / .mcap   one pair per camera stream
    <deviceId>_calibration.json       factory calibration of every virtual device
    rig.json                          the true inter-device rig
    rig_guess.json                    the same rig perturbed, a realistic initial guess for a calibration run
    ground_truth.json                 poses, intrinsics and plane the scene was rendered from
    session.json                      manifest describing the streams and the floor plane

The scene is a textured floor with boxes standing on it and a calibration chart swept through the volume all cameras
see, which is what an inter-device calibration needs: shared, textured, well spread observations.
"""
import argparse
import json
import math
import time
from datetime import timedelta
from pathlib import Path

import cv2
import numpy as np

import depthai as dai

# The rendering frame is the room: x right, y down, z forward, the floor at y == 0 and the cameras above it.
FLOOR_NORMAL = np.array([0.0, 1.0, 0.0])
BASELINE_CM = 7.5
SOCKETS = ("CAM_B", "CAM_C")


def parseArgs():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-o", "--output", type=Path, required=True, help="Directory the recording is written to")
    parser.add_argument("-n", "--frames", type=int, default=24, help="Number of frames per stream")
    parser.add_argument("-r", "--resolution", type=int, nargs=2, default=(1280, 800), help="Resolution of every camera")
    parser.add_argument("-f", "--fps", type=float, default=10.0, help="Frame rate written to the recording")
    parser.add_argument("--hfov", type=float, nargs="+", default=(72.0, 90.0, 72.0), help="Horizontal field of view of every device")
    # A 15 degree spread is what the estimation reliably matches through; wider spreads reproduce the "Not enough
    # data" / "Not enough coverage" failure of a real wide baseline rig, which is useful on purpose
    parser.add_argument("--azimuth", type=float, nargs="+", default=(-15.0, 0.0, 15.0), help="Where the devices stand around the scene, in degrees")
    parser.add_argument("--radius", type=float, default=300.0, help="Distance of the devices from the center of the scene, in cm")
    parser.add_argument("--height", type=float, default=150.0, help="Height of the devices above the floor, in cm")
    parser.add_argument("--guess-error", type=float, nargs=2, default=(8.0, 25.0), help="Rotation (deg) and translation (cm) error of rig_guess.json")
    parser.add_argument("--noise", type=float, default=2.0, help="Standard deviation of the sensor noise added to every frame")
    parser.add_argument("--seed", type=int, default=7, help="Seed of the procedural textures and the noise")
    return parser.parse_args()


# --------------------------------------------------------------------------------------------------------------- math


def normalize(vector):
    return vector / np.linalg.norm(vector)


def lookAt(center, target):
    """Camera pose (columns x right, y down, z forward) looking from center at target, as a 4x4 camera -> room matrix."""
    forward = normalize(target - center)
    right = normalize(np.cross(FLOOR_NORMAL, forward))
    down = np.cross(forward, right)
    pose = np.eye(4)
    pose[:3, :3] = np.stack([right, down, forward], axis=1)
    pose[:3, 3] = center
    return pose


def intrinsics(width, height, hfov):
    focal = 0.5 * width / math.tan(0.5 * math.radians(hfov))
    return np.array([[focal, 0.0, 0.5 * (width - 1)], [0.0, focal, 0.5 * (height - 1)], [0.0, 0.0, 1.0]])


def rotationOf(axis, angle):
    axis = normalize(np.asarray(axis, dtype=float))
    cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def asList(matrix):
    return [[float(value) for value in row] for row in np.asarray(matrix)]


# ------------------------------------------------------------------------------------------------------------- scene


class Quad:
    """A textured parallelogram: origin + two edges, sampled with the texture stretched over it."""

    def __init__(self, origin, edgeU, edgeV, texture):
        self.origin = np.asarray(origin, dtype=float)
        self.edgeU = np.asarray(edgeU, dtype=float)
        self.edgeV = np.asarray(edgeV, dtype=float)
        self.texture = texture
        self.normal = np.cross(self.edgeU, self.edgeV)

    def render(self, center, directions):
        """Intersect a bundle of rays with the quad, returning the distance along the ray and the sampled color."""
        denominator = directions @ self.normal
        with np.errstate(divide="ignore", invalid="ignore"):
            distance = ((self.origin - center) @ self.normal) / denominator
        hit = np.isfinite(distance) & (distance > 0)
        points = center + distance[..., None] * directions
        offset = points - self.origin
        u = (offset @ self.edgeU) / (self.edgeU @ self.edgeU)
        v = (offset @ self.edgeV) / (self.edgeV @ self.edgeV)
        hit &= (u >= 0) & (u <= 1) & (v >= 0) & (v <= 1)

        height, width = self.texture.shape[:2]
        mapX = np.clip(u * (width - 1), 0, width - 1).astype(np.float32)
        mapY = np.clip(v * (height - 1), 0, height - 1).astype(np.float32)
        color = cv2.remap(self.texture, mapX, mapY, cv2.INTER_LINEAR)
        return hit, np.where(hit, distance, np.inf), color


def noiseTexture(random, size, base, scale):
    """A blotchy, high frequency texture - plenty of corners for a feature detector to lock onto."""
    texture = np.zeros((size, size))
    for octave, weight in ((16, 0.5), (4, 0.35), (1, 0.3)):
        detail = random.uniform(0.0, 1.0, (max(size // octave, 1), max(size // octave, 1)))
        texture += weight * (detail if octave == 1 else cv2.resize(detail, (size, size), interpolation=cv2.INTER_CUBIC))
    gray = np.clip(base + scale * texture, 0, 255)
    return cv2.merge([gray, gray, gray]).astype(np.uint8)


def floorTexture(random, size=4096, tiles=10, marks=700):
    """A concrete-like floor with a sparse grid and scattered marks, so features are dense but not self-similar."""
    texture = noiseTexture(random, size, base=70.0, scale=60.0)
    step = size // tiles
    for index in range(tiles + 1):
        cv2.line(texture, (index * step, 0), (index * step, size), (185, 185, 185), 3)
        cv2.line(texture, (0, index * step), (size, index * step), (185, 185, 185), 3)
    for _ in range(marks):
        center = tuple(random.integers(0, size, 2).tolist())
        color = tuple(int(value) for value in random.integers(20, 240, 3))
        if random.random() < 0.5:
            cv2.circle(texture, center, int(random.integers(10, 55)), color, -1)
        else:
            extent = random.integers(15, 80, 2)
            corner = (center[0] + int(extent[0]), center[1] + int(extent[1]))
            cv2.rectangle(texture, center, corner, color, -1)
    # Landmarks that make a misalignment in the stitched view obvious at a glance
    cv2.circle(texture, (size // 2, size // 2), size // 8, (40, 40, 230), 12)
    cv2.line(texture, (size // 4, size // 4), (3 * size // 4, 3 * size // 4), (230, 120, 40), 8)
    return texture


def chartTexture(size=512, squares=8):
    texture = np.zeros((size, size, 3), dtype=np.uint8)
    step = size // squares
    for row in range(squares):
        for column in range(squares):
            if (row + column) % 2 == 0:
                texture[row * step : (row + 1) * step, column * step : (column + 1) * step] = 235
    return texture


def buildScene(random, extent):
    """The static part of the scene: the floor plus a few boxes that give the cameras structure off the plane."""
    floor = floorTexture(random)
    quads = [Quad((-extent / 2, 0.0, -extent / 2), (extent, 0.0, 0.0), (0.0, 0.0, extent), floor)]

    # Walls, so no camera looks into the void and the whole frame carries features
    wallHeight = 260.0
    corners = [(-extent / 2, -extent / 2), (extent / 2, -extent / 2), (extent / 2, extent / 2), (-extent / 2, extent / 2)]
    for index in range(4):
        start, end = corners[index], corners[(index + 1) % 4]
        wall = noiseTexture(random, 2048, base=90.0, scale=80.0)
        for _ in range(250):
            center = tuple(random.integers(0, 2048, 2).tolist())
            color = tuple(int(value) for value in random.integers(30, 235, 3))
            cv2.circle(wall, center, int(random.integers(10, 45)), color, -1)
        origin = np.array([start[0], 0.0, start[1]])
        quads.append(Quad(origin, np.array([end[0] - start[0], 0.0, end[1] - start[1]]), np.array([0.0, -wallHeight, 0.0]), wall))

    boxes = [((-160.0, -70.0), (70.0, 70.0)), ((150.0, 40.0), (60.0, 90.0)), ((0.0, 170.0), (90.0, 55.0))]
    for (x, z), (width, height) in boxes:
        texture = noiseTexture(random, 512, base=70.0, scale=110.0)
        # Four vertical faces, so the box looks solid from wherever a camera stands
        corners = [(x - width / 2, z - width / 2), (x + width / 2, z - width / 2), (x + width / 2, z + width / 2), (x - width / 2, z + width / 2)]
        for index in range(4):
            start, end = corners[index], corners[(index + 1) % 4]
            origin = np.array([start[0], 0.0, start[1]])
            quads.append(Quad(origin, np.array([end[0] - start[0], 0.0, end[1] - start[1]]), np.array([0.0, -height, 0.0]), texture))
        top = noiseTexture(random, 128, base=90.0, scale=90.0)
        quads.append(Quad(np.array([x - width / 2, -height, z - width / 2]), np.array([width, 0.0, 0.0]), np.array([0.0, 0.0, width]), top))
    return quads


def chartAt(progress, texture):
    """The calibration chart, swept through the volume the cameras share so the observations spread out."""
    angle = 2.0 * math.pi * progress
    center = np.array([110.0 * math.cos(angle), -55.0 - 35.0 * math.sin(2.0 * angle), 90.0 * math.sin(angle)])
    size = 70.0
    tilt = rotationOf((0.0, 1.0, 0.0), angle) @ rotationOf((1.0, 0.0, 0.0), math.radians(-65.0))
    edgeU = tilt @ np.array([size, 0.0, 0.0])
    edgeV = tilt @ np.array([0.0, size, 0.0])
    return Quad(center - 0.5 * (edgeU + edgeV), edgeU, edgeV, texture)


# ----------------------------------------------------------------------------------------------------------- cameras


class VirtualCamera:
    def __init__(self, deviceId, socket, poseInRoom, width, height, hfov):
        self.deviceId = deviceId
        self.socket = socket
        self.pose = poseInRoom
        self.width = width
        self.height = height
        self.intrinsics = intrinsics(width, height, hfov)
        self.hfov = hfov

        pixels = np.stack(np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64)) + [np.ones((height, width))], axis=-1)
        rays = pixels @ np.linalg.inv(self.intrinsics).T
        self.directions = rays @ self.pose[:3, :3].T
        self.center = self.pose[:3, 3]

    def render(self, quads):
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        depth = np.full((self.height, self.width), np.inf)
        for quad in quads:
            hit, distance, sampled = quad.render(self.center, self.directions)
            closer = hit & (distance < depth)
            depth = np.where(closer, distance, depth)
            color[closer] = sampled[closer]
        return color, depth

    @staticmethod
    def composite(baseColor, baseDepth, color, depth):
        closer = depth < baseDepth
        merged = baseColor.copy()
        merged[closer] = color[closer]
        return merged


def buildRig(args):
    """Place the devices around the scene and return the virtual cameras, in device order."""
    width, height = args.resolution
    cameras = []
    for index, azimuth in enumerate(args.azimuth):
        angle = math.radians(azimuth)
        center = np.array([args.radius * math.sin(angle), -args.height, -args.radius * math.cos(angle)])
        pose = lookAt(center, np.zeros(3))
        hfov = args.hfov[index % len(args.hfov)]
        deviceId = f"SYNTH{index}"
        for socketIndex, socket in enumerate(SOCKETS):
            # The stereo pair is offset along the camera's own x axis, exactly like a real board
            offset = pose.copy()
            offset[:3, 3] = pose[:3, 3] + socketIndex * BASELINE_CM * pose[:3, 0]
            cameras.append(VirtualCamera(deviceId, socket, offset, width, height, hfov))
    return cameras


# ------------------------------------------------------------------------------------------------------ calibrations


def deviceCalibration(cameras, deviceId):
    """The factory calibration of one virtual device: intrinsics of both cameras and the extrinsics between them."""
    owned = {camera.socket: camera for camera in cameras if camera.deviceId == deviceId}
    calibration = dai.CalibrationHandler()
    calibration.setDeviceName(f"SYNTH-OAK-{deviceId}")
    calibration.setProductName("synthetic multi-device rig")

    for socket, camera in owned.items():
        board = dai.CameraBoardSocket.__members__[socket]
        calibration.setCameraIntrinsics(board, asList(camera.intrinsics), camera.width, camera.height)
        calibration.setDistortionCoefficients(board, [0.0] * 14)
        calibration.setCameraType(board, dai.CameraModel.Perspective)
        calibration.setFov(board, camera.hfov)

    left, right = owned[SOCKETS[0]], owned[SOCKETS[1]]
    # Extrinsics of a camera map its own frame into the frame of `toCameraSocket`
    toRight = np.linalg.inv(right.pose) @ left.pose
    calibration.setCameraExtrinsics(
        dai.CameraBoardSocket.__members__[SOCKETS[0]],
        dai.CameraBoardSocket.__members__[SOCKETS[1]],
        asList(toRight[:3, :3]),
        [float(value) for value in toRight[:3, 3]],
        [float(value) for value in toRight[:3, 3]],
    )
    calibration.setStereoLeft(dai.CameraBoardSocket.__members__[SOCKETS[0]], asList(np.eye(3)))
    calibration.setStereoRight(dai.CameraBoardSocket.__members__[SOCKETS[1]], asList(np.eye(3)))
    return calibration


def rigOf(cameras, reference, perturbation=None, random=None):
    """The inter-device part of the geometry: every device's anchor camera expressed in the reference camera."""
    referenceCamera = next(camera for camera in cameras if (camera.deviceId, camera.socket) == reference)
    rig = dai.MultiDeviceCalibrationHandler()
    for camera in cameras:
        if camera.socket != reference[1] or camera.deviceId == reference[0]:
            continue
        transform = np.linalg.inv(referenceCamera.pose) @ camera.pose
        if perturbation is not None:
            rotationError, translationError = perturbation
            axis = random.normal(size=3)
            transform[:3, :3] = rotationOf(axis, math.radians(rotationError)) @ transform[:3, :3]
            transform[:3, 3] += translationError * normalize(random.normal(size=3))

        edge = dai.RigEdge()
        edge.from_ = dai.CoordinateFrame(camera.deviceId, dai.CameraBoardSocket.__members__[camera.socket])
        edge.to = dai.CoordinateFrame(reference[0], dai.CameraBoardSocket.__members__[reference[1]])
        edge.transform = dai.Extrinsics()
        edge.transform.setTransformationMatrix(asList(transform))
        edge.transform.setReferenceFrame(edge.to)
        edge.source = "synthetic-ground-truth" if perturbation is None else "synthetic-initial-guess"
        rig.setEdge(edge)
    return rig


# --------------------------------------------------------------------------------------------------------- recording


def main():
    args = parseArgs()
    random = np.random.default_rng(args.seed)
    width, height = args.resolution
    args.output.mkdir(parents=True, exist_ok=True)

    cameras = buildRig(args)
    deviceIds = list(dict.fromkeys(camera.deviceId for camera in cameras))
    reference = (deviceIds[0], SOCKETS[0])
    referenceCamera = next(camera for camera in cameras if (camera.deviceId, camera.socket) == reference)
    roomToReference = np.linalg.inv(referenceCamera.pose)

    print(f"Rendering {args.frames} frames of {len(cameras)} cameras at {width}x{height}", flush=True)
    # The room grows with the rig, so the cameras always stand inside it
    scene = buildScene(random, extent=max(700.0, 2.6 * args.radius))
    chart = chartTexture()
    static = [camera.render(scene) for camera in cameras]

    frames = []
    for index in range(args.frames):
        quad = chartAt(index / max(args.frames, 1), chart)
        rendered = []
        for camera, (baseColor, baseDepth) in zip(cameras, static):
            hit, distance, color = quad.render(camera.center, camera.directions)
            merged = VirtualCamera.composite(baseColor, baseDepth, color, np.where(hit, distance, np.inf))
            if args.noise > 0:
                merged = np.clip(merged + random.normal(0.0, args.noise, merged.shape), 0, 255).astype(np.uint8)
            rendered.append(merged)
        frames.append(rendered)
        print(f"  frame {index + 1}/{args.frames}", flush=True)

    calibrations = {deviceId: deviceCalibration(cameras, deviceId) for deviceId in deviceIds}
    for deviceId, calibration in calibrations.items():
        calibration.eepromToJsonFile(str(args.output / f"{deviceId}_calibration.json"))
    rigOf(cameras, reference).toJsonFile(str(args.output / "rig.json"))
    rigOf(cameras, reference, args.guess_error, random).toJsonFile(str(args.output / "rig_guess.json"))

    # The floor, expressed the way the stitching node wants it: in the reference camera frame
    planePoint = roomToReference[:3, 3]
    planeNormal = roomToReference[:3, :3] @ FLOOR_NORMAL

    streams = [
        {
            "deviceId": camera.deviceId,
            "socket": camera.socket,
            "video": f"{camera.deviceId}_{camera.socket}.avi",
            "metadata": f"{camera.deviceId}_{camera.socket}.mcap",
        }
        for camera in cameras
    ]
    manifest = {
        "reference": {"deviceId": reference[0], "socket": reference[1]},
        "resolution": [width, height],
        "fps": args.fps,
        "devices": deviceIds,
        "streams": streams,
        "hasRig": True,
        "synthetic": True,
        "plane": {"point": [float(value) for value in planePoint], "normal": [float(value) for value in planeNormal]},
    }
    (args.output / "session.json").write_text(json.dumps(manifest, indent=2))

    truth = {
        "reference": {"deviceId": reference[0], "socket": reference[1]},
        "plane": manifest["plane"],
        "cameras": [
            {
                "deviceId": camera.deviceId,
                "socket": camera.socket,
                "hfov": camera.hfov,
                "intrinsics": asList(camera.intrinsics),
                "poseInReference": asList(roomToReference @ camera.pose),
            }
            for camera in cameras
        ],
        "distances": {
            f"{a}:{SOCKETS[0]} <-> {b}:{SOCKETS[0]}": float(
                np.linalg.norm(
                    next(c for c in cameras if c.deviceId == a and c.socket == SOCKETS[0]).center
                    - next(c for c in cameras if c.deviceId == b and c.socket == SOCKETS[0]).center
                )
            )
            for index, a in enumerate(deviceIds)
            for b in deviceIds[index + 1 :]
        },
    }
    (args.output / "ground_truth.json").write_text(json.dumps(truth, indent=2))

    interval = timedelta(seconds=1.0 / args.fps)
    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        queues = []
        for camera in cameras:
            name = f"{camera.deviceId}_{camera.socket}"
            record = pipeline.create(dai.node.RecordVideo)
            record.setRecordVideoFile(args.output / f"{name}.avi")
            record.setRecordMetadataFile(args.output / f"{name}.mcap")
            # Without a frame rate the recorder waits for ten frames to estimate one, which would drop a short recording
            record.setFps(int(round(args.fps)))
            queues.append(record.input.createInputQueue())

        pipeline.start()
        for index, rendered in enumerate(frames):
            for cameraIndex, (camera, image) in enumerate(zip(cameras, rendered)):
                message = dai.ImgFrame()
                message.setCvFrame(image, dai.ImgFrame.Type.NV12)
                message.setSequenceNum(index)
                # Independent devices are not hardware synced, so the streams are offset by a fraction of the interval
                offset = interval * (0.05 * (cameraIndex // len(SOCKETS)))
                message.setTimestamp(index * interval + offset)
                message.setTimestampDevice(index * interval + offset)

                transformation = dai.ImgTransformation(camera.width, camera.height)
                transformation.setIntrinsicMatrix(asList(camera.intrinsics))
                transformation.setDistortionModel(dai.CameraModel.Perspective)
                transformation.setDistortionCoefficients([0.0] * 14)
                extrinsics = dai.Extrinsics()
                extrinsics.setTransformationMatrix(asList(np.eye(4)))
                extrinsics.setReferenceFrame(dai.CoordinateFrame(camera.deviceId, dai.CameraBoardSocket.__members__[camera.socket]))
                transformation.setExtrinsics(extrinsics)
                message.setTransformation(transformation)

                queues[cameraIndex].send(message)
        # The recorders write from their own threads, give them time to drain before the pipeline is stopped
        time.sleep(2.0 + 0.1 * args.frames)

    print(f"Wrote a synthetic recording of {len(cameras)} cameras to {args.output}", flush=True)
    for pair, distance in truth["distances"].items():
        print(f"  ground truth {pair}: {distance:.1f} cm", flush=True)


if __name__ == "__main__":
    main()
