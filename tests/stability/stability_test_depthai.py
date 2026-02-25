#!/usr/bin/env python3
import depthai as dai
import time
from typing import List, Dict
import datetime
import sys
import math

MEMORY_LEAK_DETECTION_THRESHOLD = 1.05

def calculate_latency_jitter(latencies: List[float]) -> float:
    """
    Returns jitter in seconds, defined as stddev of per-message latency.
    """
    if len(latencies) < 2:
        return 0.0

    mean_latency = sum(latencies) / len(latencies)
    variance = sum((latency - mean_latency) ** 2 for latency in latencies) / len(latencies)
    return math.sqrt(variance)


def stability_test(fps):
    # Creates the pipeline and a default device implicitly
    with dai.Pipeline() as p:
        # Define sources and outputs
        camRgb = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        monoLeft = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        neuralAssistedStereo = p.create(dai.node.NeuralAssistedStereo)
        manip = p.create(dai.node.ImageManip)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
        manip.initialConfig.setOutputSize(640,640)
        featureTracker = p.create(dai.node.FeatureTracker)
        objectTracker = p.create(dai.node.ObjectTracker)
        spatialDetectionNetwork = p.create(dai.node.SpatialDetectionNetwork).build(camRgb, neuralAssistedStereo.stereoDepth, "yolov6-nano", fps=fps)
        encoderMjpeg = p.create(dai.node.VideoEncoder)
        fullResCameraOutput = camRgb.requestFullResolutionOutput()
        encoderMjpeg.build(fullResCameraOutput, profile=dai.VideoEncoderProperties.Profile.MJPEG)

        encoderH264 = p.create(dai.node.VideoEncoder)
        encoderH264.build(fullResCameraOutput, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)

        # Stereo settings
        neuralAssistedStereo.stereoDepth.setExtendedDisparity(True)
        neuralAssistedStereo.stereoDepth.setLeftRightCheck(True)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Feature Tracker settings
        featureTracker.initialConfig.setMotionEstimator(True)
        cornerDetector = dai.FeatureTrackerConfig.CornerDetector()
        cornerDetector.numMaxFeatures = 256
        cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures
        thresholds = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
        thresholds.initialValue = 20000
        cornerDetector.thresholds = thresholds
        featureTracker.initialConfig.setCornerDetector(cornerDetector)

        # Spatial Detection Network Settings
        spatialDetectionNetwork.setConfidenceThreshold(0.6)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Linking
        fullResCameraOutput.link(manip.inputImage)
        manip.out.link(featureTracker.inputImage)
        
        spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
        spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        spatialDetectionNetwork.out.link(objectTracker.inputDetections)

        monoLeftOut = monoLeft.requestFullResolutionOutput(fps=fps)
        monoRightOut = monoRight.requestFullResolutionOutput(fps=fps)

        # Benchmark nodes
        benchmarkReportQueues : Dict[str, dai.MessageQueue] = {}
        spatialBenchmark = p.create(dai.node.BenchmarkIn)
        spatialBenchmark.logReportsAsWarnings(False)
        spatialBenchmark.measureIndividualLatencies(True)
        spatialBenchmark.sendReportEveryNMessages(fps*5)
        spatialDetectionNetwork.passthroughDepth.link(spatialBenchmark.input)
        benchmarkReportQueues["spatial"] = spatialBenchmark.report.createOutputQueue(blocking=False)

        H264Benchmark = p.create(dai.node.BenchmarkIn)
        H264Benchmark.logReportsAsWarnings(False)
        H264Benchmark.measureIndividualLatencies(True)
        H264Benchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["H264"] = H264Benchmark.report.createOutputQueue(blocking=False)
        encoderH264.out.link(H264Benchmark.input)

        MJPEGBenchmark = p.create(dai.node.BenchmarkIn)
        MJPEGBenchmark.logReportsAsWarnings(False)
        MJPEGBenchmark.measureIndividualLatencies(True)
        MJPEGBenchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["MJPEG"] = MJPEGBenchmark.report.createOutputQueue(blocking=False)
        encoderMjpeg.out.link(MJPEGBenchmark.input)

        # Neural assisted stereo
        neuralAssistedStereo.build(monoLeftOut, monoRightOut, neuralModel=dai.DeviceModelZoo.NEURAL_DEPTH_NANO)
        nasBenchmark = p.create(dai.node.BenchmarkIn)
        nasBenchmark.logReportsAsWarnings(False)
        nasBenchmark.measureIndividualLatencies(True)
        nasBenchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["N.A.Stereo"] = nasBenchmark.report.createOutputQueue(blocking=False)
        neuralAssistedStereo.depth.link(nasBenchmark.input)

        # Feature Tracker
        fTrackerBenchmark = p.create(dai.node.BenchmarkIn)
        fTrackerBenchmark.logReportsAsWarnings(False)
        fTrackerBenchmark.measureIndividualLatencies(True)
        fTrackerBenchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["Feat.Tracker"] = fTrackerBenchmark.report.createOutputQueue(blocking=False)
        featureTracker.outputFeatures.link(fTrackerBenchmark.input)

        # Object Tracker
        objTrackerBenchmark = p.create(dai.node.BenchmarkIn)
        objTrackerBenchmark.logReportsAsWarnings(False)
        objTrackerBenchmark.measureIndividualLatencies(True)
        objTrackerBenchmark.sendReportEveryNMessages(fps*5)
        benchmarkReportQueues["Obj.Tracker"] = objTrackerBenchmark.report.createOutputQueue(blocking=False)
        objectTracker.out.link(objTrackerBenchmark.input)

        # IMU
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 480)
        imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        IMUBenchmark = p.create(dai.node.BenchmarkIn)
        IMUBenchmark.logReportsAsWarnings(False)
        IMUBenchmark.measureIndividualLatencies(True)
        IMUBenchmark.sendReportEveryNMessages(30)
        imu.out.link(IMUBenchmark.input)
        benchmarkReportQueues["IMU"] = IMUBenchmark.report.createOutputQueue(blocking=False)

        print("Starting the stability test...")
        tStart = time.time()
        p.start()

        # Delay so the device process finishes starting up 
        time.sleep(10)
        initialProcessMemoryUsage = p.getDefaultDevice().getProcessMemoryUsage()
        print(f"Initial depthai-device process memory usage is: {initialProcessMemoryUsage} kB")
        while True:
            for name, queue in benchmarkReportQueues.items():
                report = queue.get(timeout=datetime.timedelta(minutes=1)) # 1 minute timeout
                if not isinstance(report, dai.BenchmarkReport):
                    print(f"{name} report is not an instance of dai.BenchmarkReport")
                if report:
                    avgLatencyMs = report.averageLatency * 1000.0
                    latencies = list(report.latencies)
                    if latencies:
                        minLatencyMs = min(latencies) * 1000.0
                        maxLatencyMs = max(latencies) * 1000.0
                        jitterMs = calculate_latency_jitter(latencies) * 1000.0
                        latencySummary = (
                            f"latency(avg/min/max): {avgLatencyMs:.2f}/{minLatencyMs:.2f}/{maxLatencyMs:.2f} ms, "
                            f"jitter(stddev): {jitterMs:.2f} ms"
                        )
                    else:
                        latencySummary = f"latency(avg): {avgLatencyMs:.2f} ms, jitter(stddev): N/A (no individual latencies attached)"
                    print(f"{name} FPS: {report.fps:.2f}, {latencySummary}. Current time: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
                    if report.fps < (fps / 10):
                        raise RuntimeError(f"FPS dropped below {fps / 10} (FPS is {report.fps}) for {name} benchmark report")
                else:
                    print(f"Timeout reached for {name} benchmark report")
                    continue
                queue.tryGetAll() # Clear the queue

            # Detect memory leaks
            processMemoryUsage = p.getDefaultDevice().getProcessMemoryUsage()
            #if processMemoryUsage > MEMORY_LEAK_DETECTION_THRESHOLD * initialProcessMemoryUsage:
            #    raise RuntimeError("Memory used by depthai-device process increased above the given threshold - potential memory leak detected")
            print(f"Memory used by depthai-device process: {processMemoryUsage} kB. Current time: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")
            print(f"Running for {datetime.timedelta(seconds=time.time() - tStart)}", flush=True)

if __name__ == "__main__":
    stability_test(30)
