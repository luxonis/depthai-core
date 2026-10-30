#!/usr/bin/env python3
"""Check RVC4 sensor selection ordering and capture with candidate host/firmware.

Example: python tests/hardware/camera_sensor_type.py --device 10.12.204.101 --sockets B C D --query-first
Use DEPTHAI_DEVICE_RVC4_FWP and PYTHONPATH to select the candidate builds.
All sockets must support --type; OG05 profile switching requires matching hardware.
"""

import argparse
import json
from datetime import timedelta

import depthai as dai


def run(args):
    sensor_type = getattr(dai.CameraSensorType, args.type)
    with dai.Device(dai.DeviceInfo(args.device)) as device, dai.Pipeline(device) as pipeline:
        device.setMaxReconnectionAttempts(0)
        assert device.getPlatform() == dai.Platform.RVC4
        sockets = [dai.CameraBoardSocket.AUTO if name == "AUTO" else getattr(dai.CameraBoardSocket, "CAM_" + name)
                   for name in args.sockets]
        initial_eeprom = {}
        if args.query_first:
            initial_eeprom = {socket: device.isCBAEepromAvailable(socket) for socket in sockets if socket != dai.CameraBoardSocket.AUTO}
            assert device.getConnectedCameraFeatures()
        cameras = []
        for socket in sockets:
            camera = pipeline.create(dai.node.Camera)
            if args.set_before_build:
                camera.setSensorType(sensor_type)
            camera.build(socket)
            if not args.set_before_build:
                camera.setSensorType(sensor_type)
            cameras.append(camera)

        # Resolving one camera prepares all sockets together. Matching selections
        # remain valid afterward; incompatible selections must fail immediately.
        assert cameras[0].getMaxWidth() > 0
        features = {f.socket: f for f in device.getConnectedCameraFeatures()}
        for socket, available in initial_eeprom.items():
            assert device.isCBAEepromAvailable(socket) == available, "CBA availability changed after camera preparation"
        for camera in cameras:
            camera.setSensorType(dai.CameraSensorType.AUTO).setSensorType(sensor_type)
            active = features[camera.getBoardSocket()]
            assert sensor_type in active.supportedTypes
            incompatible = next(t for t in (dai.CameraSensorType.MONO, dai.CameraSensorType.COLOR,
                                           dai.CameraSensorType.TOF, dai.CameraSensorType.THERMAL)
                                if t not in active.supportedTypes)
            try:
                camera.setSensorType(incompatible)
            except RuntimeError as error:
                assert "already initialized" in str(error), str(error)
            else:
                raise AssertionError("Changing the initialized sensor profile was accepted")
            assert camera.getSensorType() == sensor_type

        queues = [c.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8).createOutputQueue() for c in cameras]
        pipeline.start()
        for _ in range(20):
            for queue in queues:
                frame = queue.get(timedelta(seconds=20))
                assert frame is not None, "Timed out waiting for a camera frame"
                assert (frame.getWidth(), frame.getHeight()) == (640, 400)
                assert len(frame.getData()) > 0
        print(json.dumps({"status": "PASS", "sockets": args.sockets, "type": args.type,
                          "query_first": args.query_first, "set_before_build": args.set_before_build,
                          "frames_per_camera": 20}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--sockets", nargs="+", default=["B"], choices=["AUTO", *list("ABCDEFGH")])
    parser.add_argument("--type", default="MONO", choices=["MONO", "COLOR", "TOF", "THERMAL"])
    parser.add_argument("--query-first", action="store_true")
    parser.add_argument("--set-before-build", action="store_true")
    run(parser.parse_args())
