#!/usr/bin/env python3
"""
Multi-device calibration with explicit constraints.

multi_device_calibration.py only needs a factory stereo pair per device. This
example covers the optional MultiDeviceCalibration configuration:

  * setKnownDistance     metric scale from a measured camera-to-camera distance,
                         so a single camera per device is enough
  * setInitialGuess      an approximate rig layout as the solver's starting point
  * setDeviceCalibration calibration loaded from a file instead of the device EEPROM
  * getSampleCount       progress reporting

Example: two devices 80 cm apart, both looking at the same wall, the second one
rotated 15 degrees to the left (about the vertical axis):

  python3 multi_device_calibration_constraints.py -d <A> <B> \
      --known-distance <A> <B> 80 \
      --initial-guess <A> <B> -80 0 0  15 0 0

Device tokens are the IDs printed by the script, or whatever was passed to -d.
"""

import argparse
import math
from datetime import timedelta
from pathlib import Path

import depthai as dai


def parseSocket(name: str) -> dai.CameraBoardSocket:
    try:
        return dai.CameraBoardSocket.__members__[name.upper()]
    except KeyError:
        raise argparse.ArgumentTypeError(f"unknown camera socket {name!r}")


def rotationMatrix(yawDeg: float, pitchDeg: float, rollDeg: float) -> list:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles in degrees."""
    cy, sy = math.cos(math.radians(yawDeg)), math.sin(math.radians(yawDeg))
    cp, sp = math.cos(math.radians(pitchDeg)), math.sin(math.radians(pitchDeg))
    cr, sr = math.cos(math.radians(rollDeg)), math.sin(math.radians(rollDeg))
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def localOriginSocket(calibration: dai.CalibrationHandler, socket: dai.CameraBoardSocket) -> dai.CameraBoardSocket:
    """Follow the extrinsics chain of a device calibration to its root camera.

    MultiDeviceCalibration expresses initial guesses and results between these
    local origins, not between the sockets that were registered.
    """
    cameraData = calibration.getEepromData().cameraData
    visited = set()
    while socket in cameraData and socket not in visited:
        visited.add(socket)
        nextSocket = cameraData[socket].extrinsics.toCameraSocket
        if nextSocket == dai.CameraBoardSocket.AUTO:
            break
        socket = nextSocket
    return socket


parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("-d", "--devices", nargs="+", default=[], help="Device IDs or IPs (at least two). Defaults to the first two available devices.")
parser.add_argument("-s", "--socket", type=parseSocket, default=dai.CameraBoardSocket.CAM_A, help="Camera socket to use on every device (default: CAM_A)")
parser.add_argument("-n", "--sample-count", type=int, default=10, help="Synchronized image groups to collect before solving")
parser.add_argument("-o", "--output", type=Path, default=Path(__file__).with_name("multi_device_calibration_constraints.json"), help="Where to save the calibration JSON")
parser.add_argument(
    "--known-distance",
    nargs=3,
    action="append",
    default=[],
    metavar=("FROM", "TO", "CM"),
    help="Measured distance in cm between the camera centers of two devices (repeatable)",
)
parser.add_argument(
    "--initial-guess",
    nargs=8,
    action="append",
    default=[],
    metavar=("FROM", "TO", "X", "Y", "Z", "YAW", "PITCH", "ROLL"),
    help="Approximate transform from FROM's calibration origin to TO's: translation in cm, rotation in degrees (repeatable)",
)
parser.add_argument(
    "--calibration",
    nargs=2,
    action="append",
    default=[],
    metavar=("DEVICE", "PATH"),
    help="Use this calibration JSON for DEVICE instead of its EEPROM (repeatable)",
)
args = parser.parse_args()

if args.devices:
    if len(args.devices) < 2:
        parser.error("at least two devices are required")
    deviceInfos = [dai.DeviceInfo(d) for d in args.devices]
    tokens = list(args.devices)
else:
    deviceInfos = dai.Device.getAllAvailableDevices()[:2]
    if len(deviceInfos) < 2:
        print("At least two devices are required for this example.")
        raise SystemExit(0)
    tokens = [None] * len(deviceInfos)

socket = args.socket
calibrationFiles = {device: Path(path) for device, path in args.calibration}

with dai.Pipeline(createImplicitDevice=False) as pipeline:
    calibration = pipeline.create(dai.beta.node.MultiDeviceCalibration)
    calibration.setSampleCount(args.sample_count)
    calibration.sync.setSyncThreshold(timedelta(seconds=5))

    deviceIdByToken = {}  # user token or device ID -> device ID
    calibrations = {}  # device ID -> calibration the node will use

    for token, info in zip(tokens, deviceInfos):
        device = pipeline.addDevice(info)
        deviceId = device.getDeviceId()
        deviceIdByToken[deviceId] = deviceId
        if token is not None:
            deviceIdByToken[token] = deviceId

        # setDeviceCalibration: override the live calibration with one from a file.
        # Without an override the node reads the device calibration itself.
        calibrationFile = calibrationFiles.get(token) or calibrationFiles.get(deviceId)
        if calibrationFile is not None:
            handler = dai.CalibrationHandler(calibrationFile)
            calibration.setDeviceCalibration(deviceId, handler)
            print(f"Using device {deviceId} with calibration from {calibrationFile}")
        else:
            handler = device.readCalibration()
            print(f"Using device {deviceId}")
        calibrations[deviceId] = handler

        camera = pipeline.create(dai.node.Camera, device).build(socket, sensorFps=5)
        calibration.addCamera(deviceId, socket, camera.requestFullResolutionOutput(fps=5))

    def resolve(token: str) -> str:
        if token not in deviceIdByToken:
            parser.error(f"unknown device {token!r}; known: {sorted(set(deviceIdByToken.values()))}")
        return deviceIdByToken[token]

    # setKnownDistance: a tape-measured distance between two registered cameras on
    # different devices. With one camera per device this is the only source of scale.
    for fromToken, toToken, centimeters in args.known_distance:
        calibration.setKnownDistance(resolve(fromToken), socket, resolve(toToken), socket, float(centimeters), dai.LengthUnit.CENTIMETER)
    if not args.known_distance:
        print("Warning: no --known-distance given; with a single camera per device the solver has no metric scale.")

    # setInitialGuess: an approximate pose between two devices' local calibration
    # origins (X_to = R * X_from + t). Helps the solver when the devices are rotated
    # strongly relative to each other.
    for fromToken, toToken, x, y, z, yaw, pitch, roll in args.initial_guess:
        fromId, toId = resolve(fromToken), resolve(toToken)
        fromOrigin = localOriginSocket(calibrations[fromId], socket)
        toOrigin = localOriginSocket(calibrations[toId], socket)
        guess = dai.Extrinsics(
            rotationMatrix(float(yaw), float(pitch), float(roll)),
            dai.Point3f(float(x), float(y), float(z)),
            toOrigin,
            dai.LengthUnit.CENTIMETER,
        )
        guess.toDeviceId = toId
        calibration.setInitialGuess(fromId, fromOrigin, toId, toOrigin, guess)
        print(f"Initial guess {fromId}/{fromOrigin.name} -> {toId}/{toOrigin.name}")

    controlQueue = calibration.inputControl.createInputQueue()
    resultQueue = calibration.calibrationOutput.createOutputQueue()

    print("Point all devices at the same textured scene and keep them still.")
    pipeline.start()
    controlQueue.send(dai.beta.MultiDeviceCalibrationControl.start())
    print(f"Collecting {calibration.getSampleCount()} synchronized samples...")
    result = resultQueue.get(timedelta(minutes=3))

    if result is None or not result.passed or result.graph is None:
        raise RuntimeError(result.info if result is not None else "Calibration timed out")
    if not result.getHandler().toJsonFile(args.output):
        raise RuntimeError(f"Failed to save calibration to {args.output}")

    print(f"Calibration saved to {args.output}")
    print(f"Confidence: {result.dataConfidence:.3f}")
    print(f"Sampson error: {result.sampsonError:.6g}")
    for edge in result.graph:  # meter-normalized edges towards the reference device
        t = edge.extrinsics.translation
        distanceCm = 100.0 * math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
        print(f"{edge.fromDeviceId}/{edge.fromSocket.name} -> {edge.extrinsics.toDeviceId}/{edge.extrinsics.toCameraSocket.name}: {distanceCm:.1f} cm between origins")
