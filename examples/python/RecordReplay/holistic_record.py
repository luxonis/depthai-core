import os
import sys
import time
from pathlib import Path

import depthai as dai


def recordings_dir():
    xdg = os.environ.get("XDG_CACHE_HOME")
    if xdg:
        root = Path(xdg)
    elif sys.platform == "darwin":
        root = Path.home() / "Library" / "Caches"
    elif sys.platform == "win32":
        local = os.environ.get("LOCALAPPDATA")
        root = Path(local) if local else Path.home() / "AppData" / "Local"
    else:
        root = Path.home() / ".cache"
    path = root / "depthai" / "recordings"
    path.mkdir(parents=True, exist_ok=True)
    return path


def video_sensor(feature):
    kinds = list(feature.supportedTypes)
    if len(kinds) == 0:
        return True
    for kind in kinds:
        if kind in (dai.CameraSensorType.COLOR, dai.CameraSensorType.MONO, dai.CameraSensorType.AUTO):
            return True
    return False


def main():
    with dai.Pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        queues = []
        for feature in device.getConnectedCameraFeatures():
            if not video_sensor(feature):
                continue
            camera = pipeline.create(dai.node.Camera).build(feature.socket)
            frames = camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 30)
            queues.append(frames.createOutputQueue(4, False))
        if len(queues) == 0:
            print("No cameras found")
            return 1

        imu_name = device.getConnectedIMU()
        if imu_name and imu_name != "NONE":
            imu = pipeline.create(dai.node.IMU)
            imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
            imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
            imu.setBatchReportThreshold(20)

        output = recordings_dir()
        rvc4 = device.getPlatform() == dai.Platform.RVC4
        device_before = set()
        have_device_list = False
        if rvc4:
            try:
                device_before = {info.name for info in device.listRecordings()}
                have_device_list = True
            except Exception:
                device_before = set()

        config = dai.RecordConfig()
        config.outputDir = output
        pipeline.enableHolisticRecord(config)
        print(f"Recording to {output}")
        pipeline.start()
        deadline = time.monotonic() + 5
        if rvc4 and have_device_list:
            look_until = time.monotonic() + 2
            printed = False
            while not printed and time.monotonic() < look_until and pipeline.isRunning():
                try:
                    for info in device.listRecordings():
                        if info.name in device_before:
                            continue
                        print(f"/data/depthai/recordings/{info.name}")
                        printed = True
                except Exception:
                    break
                if not printed:
                    time.sleep(0.1)
        while time.monotonic() < deadline and pipeline.isRunning():
            time.sleep(0.05)
        pipeline.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
