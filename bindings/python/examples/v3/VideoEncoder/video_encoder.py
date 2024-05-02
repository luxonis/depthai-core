import depthai as dai

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
            videoSize = (3840, 2160))
    encoded = dai.node.VideoEncoder(camRgb.video,
            frameRate = 30, 
            profile = dai.VideoEncoderProperties.Profile.H265_MAIN)
    saver = VideoSaver(encoded.out)

    pipeline.start()
    pipeline.wait()
    # TODO Close file handle

print("To view the encoded data, convert the stream file (.h265) into a video file (.mp4) using a command below:")
print("ffmpeg -framerate 30 -i video.h265 -c copy video.mp4")
