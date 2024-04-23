import depthai as dai
import argparse
import time
import signal


parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default="test_video", help="Output file name (without extension)")

args = parser.parse_args()

with dai.Pipeline() as pipeline:
    def signal_handler(sig, frame):
        print("Interrupted, stopping the pipeline")
        pipeline.stop()
    signal.signal(signal.SIGINT, signal_handler)

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    videoEncoder = pipeline.create(dai.node.VideoEncoder)
    videoEncoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)

    record = pipeline.create(dai.node.Record)
    record.setRecordFile(args.output)

    cam.video.link(videoEncoder.input)
    videoEncoder.out.link(record.input)

    pipeline.start()
    print("Recording video. Press Ctrl+C to stop.")
    while pipeline.isRunning():
        time.sleep(1)

