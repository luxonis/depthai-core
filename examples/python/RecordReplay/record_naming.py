# Showcases multiple record nodes with different names

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


def main():
    with dai.Pipeline() as pipeline:
        directory = recordings_dir()
        name = "naming_" + dai.makeRecordingTimestamp()
        record = None
        for socket, stream in ((dai.CameraBoardSocket.CAM_A, "CAM_A"), (dai.CameraBoardSocket.CAM_B, "CAM_B")):
            camera = pipeline.create(dai.node.Camera).build(socket)
            frames = camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 10)
            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)
            frames.link(encoder.input)

            record = pipeline.create(dai.node.RecordVideo)
            record.setKeepOnDevice(False)
            record.setFps(10)
            record.setOutputDirectory(directory)
            record.setStreamName(stream)
            record.setRecordingName(name)
            encoder.out.link(record.input)

        print(f"Saving video to {directory / name}")
        if pipeline.getDefaultDevice().getPlatform() == dai.Platform.RVC4:
            print(f"/data/depthai/recordings/{name}")
        pipeline.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and pipeline.isRunning():
            time.sleep(0.05)
        pipeline.stop()
        if record is not None:
            last = record.getStatus()
            if last is not None:
                print(f"frames={last.frameCount} path={last.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
