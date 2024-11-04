import depthai as dai

# Capture Ctrl+C and set a flag to stop the loop
import time
import cv2
import threading
import signal

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

with dai.Pipeline() as pipeline:
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    output = camRgb.requestOutput((1920, 1440), type=dai.ImgFrame.Type.NV12)
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
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
    pipeline.stop()
    pipeline.wait()
    saver.file_handle.close()

print("To view the encoded data, convert the stream file (.encoded) into a video file (.mp4) using a command below:")
print("ffmpeg -framerate 30 -i video.encoded -c copy video.mp4")
