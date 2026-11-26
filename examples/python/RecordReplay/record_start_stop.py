import os
import sys
from pathlib import Path

import cv2
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
    fps = 30
    with dai.Pipeline() as pipeline:
        camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        frames = camera.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, fps)
        video_queue = frames.createOutputQueue()

        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)
        frames.link(encoder.input)

        record = pipeline.create(dai.node.RecordVideo)
        encoder.out.link(record.input)
        name = "start-stop_" + dai.makeRecordingTimestamp()
        directory = recordings_dir()
        record.setOutputDirectory(directory)
        record.setRecordingName(name)
        record.setFps(fps)
        record.setWaitForTrigger(True)
        control = record.inputControl.createInputQueue()

        print(f"Saving video to {directory / name}")
        if pipeline.getDefaultDevice().getPlatform() == dai.Platform.RVC4:
            print(f"/data/depthai/recordings/{name}")
        print("Press s to start or stop recording. Press q to quit.")

        pipeline.start()
        recording = False
        while pipeline.isRunning():
            try:
                frame = video_queue.get()
            except dai.MessageQueue.QueueException:
                break
            if frame is None:
                continue
            cv2.imshow("video", frame.getCvFrame())
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                if recording:
                    control.send(dai.RecordControl.stop())
                    print("stop")
                    recording = False
                else:
                    control.send(dai.RecordControl.start())
                    print("start")
                    recording = True
        pipeline.stop()
        last = record.getStatus()
        if last is not None:
            print(f"frames={last.frameCount} path={last.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
