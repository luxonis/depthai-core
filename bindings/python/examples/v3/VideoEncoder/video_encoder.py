import depthai as dai

# Capture Ctrl+C and set a flag to stop the loop
import signal
import sys
import time

stopped = False
def signal_handler(sig, frame):
    global stopped
    print('You pressed Ctrl+C!, stopping video saving...')
    stopped = True

signal.signal(signal.SIGINT, signal_handler)
class VideoSaver(dai.node.HostNode):
    def __init__(self, *args):
        dai.node.HostNode.__init__(self)
        self.link_args(*args)
        self.file_handle = open('video.h265', 'wb')

    def process(self, frame: dai.EncodedFrame):
        frame.getData().tofile(self.file_handle)

with dai.Pipeline() as pipeline:
    camRgb = dai.node.Camera(
            boardSocket = dai.CameraBoardSocket.CAM_A,
            videoSize = (1920, 1080))
    encoded = dai.node.VideoEncoder(camRgb.video,
            frameRate = 30,
            profile = dai.VideoEncoderProperties.Profile.H265_MAIN)
    saver = VideoSaver(encoded.out)

    pipeline.start()
    print("Started to save video to video.h265")
    print("Press Ctrl+C to stop")
    while pipeline.isRunning() and not stopped:
        time.sleep(0.1)
    pipeline.stop()
    pipeline.wait()
    # TODO Close file handle

print("To view the encoded data, convert the stream file (.h265) into a video file (.mp4) using a command below:")
print("ffmpeg -framerate 30 -i video.h265 -c copy video.mp4")
