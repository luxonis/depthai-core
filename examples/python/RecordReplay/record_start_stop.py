# Showcases starting and stopping the recording from host
# Note the usage of H.264 when running on host which creates segments of 30 decodable frames
# and may need a pretrigger buffer to work correctly

import os
import time
from pathlib import Path

import depthai as dai

import example_util as rr
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument('--pretrigger-size', type=int, default=0, help='The amount of frames kept on standby while the node is not recording. Used to see what happened a few seconds before the recording started.')
parser.add_argument('--cycles', type=int, default=1, help='number of segments recorded')
parser.add_argument('--segment-duration-limit', type=float, default=-1, help='a limit on the length of a segment independent of a stop command')
parser.add_argument('--segment-frames-limit', type=int, default=-1, help='a limit on the amount of frames a segment can have independent of a stop command')
parser.add_argument('--segment-interval', type=float, default=1, help='time between the STOP and the START command')
parser.add_argument('--segment-duration', type=float, default=4, help='time between the START and the STOP command')
parser.add_argument('--fps', type=int, default=10)
args = parser.parse_args()

def main():
    pipeline = None
    record = None
    fps = args.fps
    cycles = args.cycles
    pretriggerFrames = args.pretrigger_size
    duration = args.segment_duration
    interval = args.segment_interval
    maxFrames = args.segment_frames_limit
    maxDuration = args.segment_duration_limit
    try:
        pipeline = dai.Pipeline()
        frames = rr.add_camera_nv12(pipeline, fps)
        record = pipeline.create(dai.node.RecordVideo)
        rr.link_encoder(pipeline, frames, record)
        stem = rr.output_dir() / rr.unique_name('start-stop')
        record.setOutputDirectory(stem)
        print(f'Saving video to "{os.fspath(record.getRecordVideoFile())}"')
        print(f'Saving metadata to "{os.fspath(record.getRecordMetadataFile())}"')
        record.setFps(fps)
        if (pretriggerFrames < 0):
            record.setWaitForTrigger(False)
        else:
            record.setPreTriggerFrames(pretriggerFrames)
            record.setWaitForTrigger(True)
        control = record.inputControl.createInputQueue()
        status = record.status.createOutputQueue(16, False)
        print(f'RecordVideo runOnHost={record.runOnHost()}')
        code = rr.start_or_classify(pipeline)
        if code != 0:
            return code
        for i in range(cycles):
            time.sleep(interval)
            print('sending start()')
            control.send(dai.RecordControl.start(maxFrames, maxDuration*1000))
            time.sleep(duration)
            print('sending stop()')
            control.send(dai.RecordControl.stop())
        time.sleep(0.5)
        # calls pipeline.stop and prints the backlog of record node statuses
        rr.finish_recording(pipeline, record, status)
    except Exception as ex:
        return rr.classify_record_failure(str(ex))
    finally:
        record = None
        frames = None
        control = None
        status = None
        if pipeline is not None:
            pipeline.stop()


if __name__ == '__main__':
    raise SystemExit(main())
