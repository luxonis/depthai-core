import depthai as dai

# Capture Ctrl+C and set a flag to stop the loop
import signal
import time
import cv2


PROFILE = dai.VideoEncoderProperties.Profile.MJPEG
stopped = False

def signal_handler(sig, frame):
    global stopped
    print('You pressed Ctrl+C!, stopping video saving...')
    stopped = True
signal.signal(signal.SIGINT, signal_handler)

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
    camRgb = pipeline.create(dai.node.Camera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    output = camRgb.requestOutput((1920, 1440))
    outputQueue = output.createOutputQueue()
    encoded = pipeline.create(dai.node.VideoEncoder).build(output,
            frameRate = 30,
            profile = PROFILE)
    saver = pipeline.create(VideoSaver).build(encoded.bitstream)

    pipeline.start()
    print("Started to save video to video.encoded")
    print("Press Ctrl+C to stop")
    timeStart = time.monotonic()
    while pipeline.isRunning() and not stopped:
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
