import depthai as dai
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-v", "--video", default="recordings/recording.mp4", help="Path to the recorded video")
parser.add_argument("-s", "--source", default="recordings/recording.mcap", help="Path to the recorded mcap file")
args = parser.parse_args()

is_video = args.video is not None

with dai.Pipeline() as pipeline:
    cam = pipeline.create
    if is_video:
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        camRgbOut = camRgb.requestOutput((1920, 1080), fps = 30)
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(args.video))
        replay.setReplayMetadataFile(Path(args.source))
        replay.setOutFrameType(dai.ImgFrame.Type.NV12)
        replay.out.link(camRgb.mockIsp)
        camQ = camRgbOut.createOutputQueue()
        replayQ = replay.out.createOutputQueue()
        pipeline.start()
        try:
            frame1 = camQ.get()
            frame2 = replayQ.get()
            while True:
                if(frame1.getSequenceNum() > frame2.getSequenceNum()):
                    frame2 = replayQ.get()
                elif(frame1.getSequenceNum() < frame2.getSequenceNum()):
                    frame1 = camQ.get()
                else:
                    if(frame1.getTimestampDevice() != frame2.getTimestampDevice()):
                        print("Timestamps do not match")

        except KeyboardInterrupt:
            pipeline.stop()
    else:
        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500);
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400);
        replay = pipeline.create(dai.node.ReplayMetadataOnly)
        replay.setReplayFile(Path(args.source))
        replay.out.link(imu.mockIn)
        imuQ = imu.out.createOutputQueue()
        replayQ = replay.out.createOutputQueue()
        pipeline.start()
        try:
            frame1 = imuQ.get()
            frame2 = replayQ.get()
            while True:
                if(frame1.getSequenceNum() > frame2.getSequenceNum()):
                    frame2 = replayQ.get()
                elif(frame1.getSequenceNum() < frame2.getSequenceNum()):
                    frame1 = imuQ.get()
                else:
                    if(frame1.getTimestampDevice() != frame2.getTimestampDevice()):
                        print("Timestamps do not match")
        except KeyboardInterrupt:
            pipeline.stop()
