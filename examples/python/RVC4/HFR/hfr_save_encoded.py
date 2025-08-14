import depthai as dai

# Capture Ctrl+C and set a flag to stop the loop
import time
import cv2
import threading
import signal

PROFILE = dai.VideoEncoderProperties.Profile.H264_MAIN

quitEvent = threading.Event()
signal.signal(signal.SIGTERM, lambda *_args: quitEvent.set())
signal.signal(signal.SIGINT, lambda *_args: quitEvent.set())

SIZE = (1280, 720)
FPS = 480

# SIZE = (1920, 1080)
# FPS = 240

class VideoSaver(dai.node.HostNode):
    def __init__(self, *args, **kwargs):
        dai.node.HostNode.__init__(self, *args, **kwargs)
        self.file_handle = open('video_hfr.encoded', 'wb')

    def build(self, *args):
        self.link_args(*args)
        return self

    def process(self, frame):
        frame.getData().tofile(self.file_handle)

with dai.Pipeline() as pipeline:
    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    output = camRgb.requestOutput(SIZE, fps=FPS)

    # ImageManip is added to workaround a limitation with VideoEncoder with native resolutions
    # This limitation will be lifted in the future
    imageManip = pipeline.create(dai.node.ImageManip)
    imageManip.initialConfig.setOutputSize(SIZE[0], SIZE[1] + 10) # To avoid a passthrough
    imageManip.setMaxOutputFrameSize(int(SIZE[0] * (SIZE[1] + 10) * 1.6))
    output.link(imageManip.inputImage)
    output = imageManip.out

    benchmarkIn = pipeline.create(dai.node.BenchmarkIn)
    benchmarkIn.setRunOnHost(True)

    encoded = pipeline.create(dai.node.VideoEncoder).build(output,
            frameRate = FPS,
            profile = PROFILE)
    encoded.out.link(benchmarkIn.input)
    saver = pipeline.create(VideoSaver).build(encoded.out)

    pipeline.start()
    print("Started to save video to video.encoded")
    print("Press Ctrl+C to stop")
    timeStart = time.monotonic()
    while pipeline.isRunning() and not quitEvent.is_set():
        time.sleep(1)
    pipeline.stop()
    pipeline.wait()
    saver.file_handle.close()

print("To view the encoded data, convert the stream file (.encoded) into a video file (.mp4) using a command below:")
print(f"ffmpeg -framerate {FPS} -i video_hfr.encoded -c copy video_hfr.mp4")

print("If the FPS is not set correctly, you can ask ffmpeg to generate it with the command below")

print(f"""
ffmpeg -fflags +genpts -r {FPS} -i video_hfr.encoded \\
  -vsync cfr -fps_mode cfr \\
  -video_track_timescale {FPS}00 \\
  -c:v copy \\
  video_hfr.mp4
""")
