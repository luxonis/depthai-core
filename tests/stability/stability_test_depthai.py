#!/usr/bin/env python3
import depthai as dai
import time
from typing import List, Dict
import datetime

def stability_test(fps):
    # Creates the pipeline and a default device implicitly
    with dai.Pipeline() as p:
        # Define sources and outputs
        camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = p.create(dai.node.StereoDepth)
        spatialDetectionNetwork = p.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, "yolov6-nano", fps=fps)
        encoderMjpeg = p.create(dai.node.VideoEncoder)
        fullResCameraOutput = camRgb.requestFullResolutionOutput()
        encoderMjpeg.build(fullResCameraOutput, profile=dai.VideoEncoderProperties.Profile.MJPEG)

        encoderH264 = p.create(dai.node.VideoEncoder)
        encoderH264.build(fullResCameraOutput, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)

        # Stereo settings
        stereo.setExtendedDisparity(True)
        stereo.setLeftRightCheck(True)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Linking
        monoLeft.requestFullResolutionOutput(fps=fps).link(stereo.left)
        monoRight.requestFullResolutionOutput(fps=fps).link(stereo.right)

        # Benchmark nodes
        benchmarkReportQueues : Dict[str, dai.MessageQueue] = {}
        spatialBenchmark = p.create(dai.node.BenchmarkIn)
        spatialBenchmark.logReportsAsWarnings(False)
        spatialBenchmark.sendReportEveryNMessages(fps*5)
        spatialDetectionNetwork.passthroughDepth.link(spatialBenchmark.input)
        benchmarkReportQueues["spatial"] = spatialBenchmark.report.createOutputQueue(blocking=False)

        H264Benchmark = p.create(dai.node.BenchmarkIn)
        H264Benchmark.logReportsAsWarnings(False)
        H264Benchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["H264"] = H264Benchmark.report.createOutputQueue(blocking=False)
        encoderH264.out.link(H264Benchmark.input)

        MJPEGBenchmark = p.create(dai.node.BenchmarkIn)
        MJPEGBenchmark.logReportsAsWarnings(False)
        MJPEGBenchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["MJPEG"] = MJPEGBenchmark.report.createOutputQueue(blocking=False)
        encoderMjpeg.out.link(MJPEGBenchmark.input)

        # IMU
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 480)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        IMUBenchmark = p.create(dai.node.BenchmarkIn)
        IMUBenchmark.logReportsAsWarnings(False)
        IMUBenchmark.sendReportEveryNMessages(30)
        imu.out.link(IMUBenchmark.input)
        benchmarkReportQueues["IMU"] = IMUBenchmark.report.createOutputQueue(blocking=False)
        p.start()
        while True:
            for name, queue in benchmarkReportQueues.items():
                report = queue.get(timeout=datetime.timedelta(minutes=1)) # 1 minute timeout
                assert(isinstance(report, dai.BenchmarkReport))
                if report:
                    print(f"{name} FPS: {report.fps}. Current date: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
                    if report.fps < (fps / 10):
                        raise RuntimeError(f"FPS dropped below {fps / 10} (FPS is {report.fps}) for {name} benchmark report")
                else:
                    raise RuntimeError(f"Timeout reached for {name} benchmark report")
                queue.tryGetAll() # Clear the queue

if __name__ == "__main__":
    stability_test(30)