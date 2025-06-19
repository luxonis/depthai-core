import argparse
import subprocess
import sys
import time
import threading
import signal

import depthai as dai
import cv2
import pyfakewebcam

PROFILE = dai.VideoEncoderProperties.Profile.MJPEG # or H265_MAIN, H264_MAIN

quitEvent = threading.Event()
signal.signal(signal.SIGTERM, lambda *_args: quitEvent.set())
signal.signal(signal.SIGINT, lambda *_args: quitEvent.set())

class VideoSaver(dai.node.HostNode):
    def __init__(self, *args, **kwargs):
        dai.node.HostNode.__init__(self, *args, **kwargs)
        self.file_handle = open('video.encoded', 'wb')

    def build(self, *args):
        self.link_args(*args)
        return self

    def process(self, frame):
        frame.getData().tofile(self.file_handle)


def load_v4l2loopback(device: str = "/dev/video0"):
    """
    Load the v4l2loopback kernel module and create a loopback device.
    """
    # Parameters to configure the loopback device
    module_args = [
        "devices=1",
        f"video_nr={device.replace('/dev/video', '')}",
        "card_label=LoopbackCam",
        "exclusive_caps=1"
    ]
    cmd = ["sudo", "modprobe", "v4l2loopback"] + module_args
    try:
        subprocess.run(cmd, check=True)
        print(f"Loaded v4l2loopback with device {device}")
    except subprocess.CalledProcessError as e:
        print(f"Error loading v4l2loopback module: {e}", file=sys.stderr)
        sys.exit(1)


def init_pyfakewebcam(device: str, width: int, height: int, fps: float = 30.0):
    interval = 1.0 / fps

    # Initialize fake webcam
    fake = pyfakewebcam.FakeWebcam(device, width, height)
    print(f"Streaming {mjpeg_path} ({width}x{height}@{fps:.2f}fps) to {device}")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # Loop the video by resetting the capture position
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue

            # OpenCV uses BGR; pyfakewebcam expects RGB
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            fake.schedule_frame(rgb)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Interrupted by user, exiting...")
    finally:
        cap.release()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream an MJPEG file into a v4l2loopback device"
    )
    parser.add_argument(
        "--device",
        default="/dev/video0",
        help="v4l2loopback device (e.g., /dev/video0)"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    load_v4l2loopback(args.device)
    # mjpeg_to_loopback(args.mjpeg, args.device)
    fake = pyfakewebcam.FakeWebcam(args.device, 1920, 1080)

    with dai.Pipeline() as pipeline:
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        output = camRgb.requestOutput((1920, 1080), type=dai.ImgFrame.Type.NV12)
        outputQueue = output.createOutputQueue()
        encoded = pipeline.create(dai.node.VideoEncoder).build(output,
                frameRate = 30,
                profile = PROFILE)
        saver = pipeline.create(VideoSaver).build(encoded.out)

        pipeline.start()
        print("Started to save video to video.encoded")
        print("Press Ctrl+C to stop")
        timeStart = time.monotonic()
        while pipeline.isRunning() and not quitEvent.is_set():
            frame = outputQueue.get()
            assert isinstance(frame, dai.ImgFrame)
            cv2.imshow("video", frame.getCvFrame())
            frame_rgb = cv2.cvtColor(frame.getCvFrame(), cv2.COLOR_BGR2RGB)
            fake.schedule_frame(frame_rgb)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
        pipeline.stop()
        pipeline.wait()
        saver.file_handle.close()



if __name__ == "__main__":
    main()