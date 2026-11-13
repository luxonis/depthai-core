# Showcases record node configuration and basic usage.
# On RVC4 the node records into the device store and, at pipeline stop, if
# --keep-on-device is false copies the video to the .mp4 path and the metadata to the .mcap path.

import os
from pathlib import Path

import depthai as dai

import example_util as rr
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--duration", type=int, default=2)
parser.add_argument("--keep-on-device", action='store_true')
parser.add_argument("--fps", type=int, default=10)
parser.add_argument("--codec", type=str, default="H264_MAIN", help="A codec supported by videoEncoder or none for RAW_FRAMES")
args = parser.parse_args()


def main():
    duration = args.duration
    onDevice = args.keep_on_device
    codec = None
    if (args.codec != "none"):
        codecs = dai.VideoEncoderProperties.Profile.__members__
        if (args.codec not in codecs):
            print(f'Invalid codec {args.codec} specified. Use one of the following: {list(codecs)}')
            exit(1)
        codec = codecs[args.codec]
    fps = args.fps

    pipeline = None
    try:
        pipeline = dai.Pipeline()
        frames = rr.add_camera_nv12(pipeline, fps)
        record = pipeline.create(dai.node.RecordVideo)
        if (codec is None):
            record.setFormat(dai.RecordFormat.RAW_FRAMES)
            frames.link(record.input)
        else:
            rr.link_encoder(pipeline, frames, record, codec)
        if (not onDevice):
            stem = rr.output_dir() / rr.unique_name("basic")
            record.setOutputDirectory(stem)
            print(f'Saving {"raw frames" if codec is None else "video"} to "{os.fspath(record.getOutputDirectory())}"')
        record.setKeepOnDevice(onDevice)
        record.setFps(fps)
        print(f"Starting recording with {args.codec} for {duration}s at {fps}fps.")
        print(f"The recording will be {"kept on device" if onDevice else "tranferred to host"}.")
        # calls pipeline.start and prints record node status
        return rr.record_for(pipeline, record, duration)
    except Exception as ex:
        return rr.classify_record_failure(str(ex))
    finally:
        record = None
        frames = None
        if pipeline is not None:
            pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
