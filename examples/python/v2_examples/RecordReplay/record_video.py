import depthai as dai
import argparse
from time import sleep

parser = argparse.ArgumentParser()
parser.add_argument("-o", "--output", default="video.mp4", help="Output file name")

args = parser.parse_args()

pipeline = dai.Pipeline()

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

sleep(10)

pipeline.stop()

