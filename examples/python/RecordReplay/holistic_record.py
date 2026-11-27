# Showcases Pipeline.enableHolisticRecord.
# Holistic record attaches a RecordVideo to every Camera source, and a
# RecordMetadataOnly to the IMU, when the pipeline starts. This script does not
# create those nodes. requestOutput sets the size and fps that holistic record
# copies, and the queue keeps that request linked. An unlinked request is
# rejected at start.

import os
import time
from pathlib import Path

import depthai as dai

import example_util as rr
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--duration", type=float, default=5, help="seconds to record after every source has produced a message")
parser.add_argument("--fps", type=float, default=10)
parser.add_argument("--width", type=int, default=640, help="recorded width. 640 is a multiple of 32, which RVC2 encoding requires")
parser.add_argument("--height", type=int, default=480, help="recorded height. 480 is a multiple of 8, which RVC2 encoding requires")
parser.add_argument("--codec", type=str, default="H264_MAIN", help="a VideoEncoder profile, or none for RAW_FRAMES")
parser.add_argument("--compression", type=str, default="NONE", help="mcap compression: NONE, FASTEST, FAST, DEFAULT, SLOW, SLOWEST")
parser.add_argument("--bitrate", type=int, default=0, help="encoder bitrate. 0 keeps the encoder default")
parser.add_argument("--quality", type=int, default=80, help="encoder quality, used by MJPEG")
parser.add_argument("--lossless", action="store_true", help="ask the encoder for a lossless stream")
parser.add_argument("--keep-on-device", action="store_true", help="keep the RVC4 device copy after the host archive is packed")
parser.add_argument("--no-sync", action="store_true", help="record each camera on its own clock instead of Sync")
parser.add_argument("--imu-rate", type=int, default=200, help="accelerometer and gyroscope report rate, used only when an IMU is connected")
args = parser.parse_args()

def video_sensor(feature):
    types = list(feature.supportedTypes)
    if len(types) == 0:
        return True
    for sensor_type in types:
        if sensor_type in (dai.CameraSensorType.COLOR, dai.CameraSensorType.MONO, dai.CameraSensorType.AUTO):
            return True
    return False


def imu_connected(name):
    return name != "" and name != "NONE"


def main():
    pipeline = None
    device = None
    cameras = []
    queues = []
    imu = None
    imu_queue = None
    try:
        # parse arguments
        codec = None
        if args.codec != "none":
            codecs = dai.VideoEncoderProperties.Profile.__members__
            if args.codec not in codecs:
                print(f"Invalid codec {args.codec}. Use one of {list(codecs)}, or none")
                return 1
            codec = codecs[args.codec]
        levels = dai.RecordConfig.CompressionLevel.__members__
        if args.compression not in levels:
            print(f"Invalid compression {args.compression}. Use one of {list(levels)}")
            return 1

        # initialize pipeline
        pipeline = dai.Pipeline()
        device = pipeline.getDefaultDevice()
        features = device.getConnectedCameraFeatures()
        if len(features) == 0:
            print("No cameras found")
            return 1

        # configure holistic record
        output = rr.output_dir() / rr.unique_name("holistic")
        output.mkdir(parents=True, exist_ok=True)
        config = dai.RecordConfig()
        config.outputDir = output
        config.keepOnDevice = args.keep_on_device
        config.compressionLevel = levels[args.compression]
        config.syncCameraOutputs = not args.no_sync
        config.videoEncoding.bitrate = args.bitrate
        config.videoEncoding.quality = args.quality
        config.videoEncoding.lossless = args.lossless
        if codec is None:
            config.format = dai.RecordFormat.RAW_FRAMES
            config.videoEncoding.enabled = False
        else:
            config.format = dai.RecordFormat.VIDEO
            config.videoEncoding.enabled = True
            config.videoEncoding.profile = codec
        pipeline.enableHolisticRecord(config)

        # Create an output queue for each camera on the device, so they are all captured.
        # Otherwise holistic record only records used cameras
        size = (args.width, args.height)
        for feature in features:
            label = f"{feature.socket} {feature.sensorName} {feature.width}x{feature.height}"
            if not video_sensor(feature):
                print(f"Skipping {label}: holistic record requests NV12")
                continue
            camera = pipeline.create(dai.node.Camera).build(feature.socket)
            queue = camera.requestOutput(size, dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, args.fps).createOutputQueue(4, False)
            cameras.append(camera)
            queues.append(queue)
            print(f"Recording {label} at {args.width}x{args.height}")
        if len(cameras) == 0:
            print("No color or mono camera to record")
            return 1

        # Record IMU if exists
        imu_name = device.getConnectedIMU()
        if imu_connected(imu_name):
            imu = pipeline.create(dai.node.IMU)
            imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, args.imu_rate)
            imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, args.imu_rate)
            imu.setBatchReportThreshold(20)
            imu_queue = imu.out.createOutputQueue(4, False)
            print(f"Recording IMU {imu_name} at {args.imu_rate} Hz")
        else:
            print(f"No IMU on this device ({imu_name or 'empty'}). IMU will not be recorded.")

        # start pipeline
        mode = "RAW_FRAMES" if codec is None else args.codec
        print(
            f"Starting holistic record {mode} compression={args.compression} "
            f"for {args.duration}s at {args.fps} fps, sync={not args.no_sync}."
        )
        print(f'Archive directory "{os.fspath(output)}"')
        pipeline.start()

        # Wait for cameras and IMU to start
        ready = [False] * len(queues)
        imu_ready = imu_queue is None
        deadline = time.monotonic() + 10
        while (not all(ready) or not imu_ready) and pipeline.isRunning() and time.monotonic() < deadline:
            for index, queue in enumerate(queues):
                if rr.try_get(queue, dai.ImgFrame) is not None:
                    ready[index] = True
            if imu_queue is not None and rr.try_get(imu_queue, dai.IMUData) is not None:
                imu_ready = True
            time.sleep(0.05)
        if not all(ready) or not imu_ready:
            raise RuntimeError("A camera or the IMU did not produce data during holistic recording")

        # Print frame count every second.
        counts = [0] * len(queues)
        imu_count = 0
        end = time.monotonic() + args.duration
        next_print = time.monotonic() + 1
        while time.monotonic() < end and pipeline.isRunning():
            for index, queue in enumerate(queues):
                if rr.try_get(queue, dai.ImgFrame) is not None:
                    counts[index] += 1
            if imu_queue is not None and rr.try_get(imu_queue, dai.IMUData) is not None:
                imu_count += 1
            if time.monotonic() >= next_print:
                summary = " ".join(f"cam{index}={count}" for index, count in enumerate(counts))
                if imu_queue is not None:
                    summary += f" imu={imu_count}"
                print(summary)
                next_print += 1
            time.sleep(0.05)
        if not pipeline.isRunning():
            raise RuntimeError("Pipeline stopped before the recording duration elapsed")

        # stop pipeline and archive outputs
        pipeline.stop()
        archive = None
        for entry in output.iterdir():
            if entry.is_file() and entry.suffix == ".gz" and Path(entry.stem).suffix == ".tar" and entry.stat().st_size > 0:
                archive = entry
        if archive is None:
            raise RuntimeError("Holistic recording did not create a nonempty archive")
        print(f'holistic record stopped, archive "{os.fspath(archive)}"')
        return 0
    except Exception as ex:
        return rr.classify_record_failure(str(ex))
    finally:
        cameras = None
        queues = None
        imu = None
        imu_queue = None
        if pipeline is not None:
            try:
                if pipeline.isRunning():
                    pipeline.stop()
            except Exception:
                pass
        pipeline = None
        if device is not None and not device.isClosed():
            device.close()
        device = None


if __name__ == "__main__":
    raise SystemExit(main())
