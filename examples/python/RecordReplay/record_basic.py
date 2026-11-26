# Showcases record node configuration and basic usage.
# On RVC4 the node records into the device store and, at pipeline stop, if
# --keep-on-device is false copies the video to the host directory and the metadata beside it.

import argparse
import os
import sys
import time
from pathlib import Path

import depthai as dai

parser = argparse.ArgumentParser()
parser.add_argument("--duration", type=int, default=2)
parser.add_argument("--keep-on-device", action="store_true")
parser.add_argument("--fps", type=int, default=10)
parser.add_argument("--codec", type=str, default="H264_MAIN", help="A codec supported by videoEncoder or none for RAW_FRAMES")
args = parser.parse_args()


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
    codec = None
    if args.codec != "none":
        codecs = dai.VideoEncoderProperties.Profile.__members__
        if args.codec not in codecs:
            print(f"Invalid codec {args.codec} specified. Use one of the following: {list(codecs)}")
            return 1
        codec = codecs[args.codec]

    with dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        frames = camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, args.fps)
        record = pipeline.create(dai.node.RecordVideo)
        if codec is None:
            record.setFormat(dai.RecordFormat.RAW_FRAMES)
            frames.link(record.input)
        else:
            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setProfile(codec)
            frames.link(encoder.input)
            encoder.out.link(record.input)

        name = "basic_" + dai.makeRecordingTimestamp()
        directory = recordings_dir()
        record.setOutputDirectory(directory)
        record.setRecordingName(name)
        record.setKeepOnDevice(args.keep_on_device)
        record.setFps(args.fps)
        print(f'Saving {"raw frames" if codec is None else "video"} to "{directory / name}"')
        if pipeline.getDefaultDevice().getPlatform() == dai.Platform.RVC4:
            print(f"/data/depthai/recordings/{name}")
        print(f"Starting recording with {args.codec} for {args.duration}s at {args.fps}fps.")
        print(f"The recording will be {'kept on device' if args.keep_on_device else 'transferred to host'}.")

        pipeline.start()
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and pipeline.isRunning():
            time.sleep(0.05)
        pipeline.stop()
        last = record.getStatus()
        if last is not None:
            print(f"frames={last.frameCount} path={last.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
