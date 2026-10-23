#!/usr/bin/env python3
"""Exercise example pipelines repeatedly on one caller-owned RVC4 device.

Run --list without hardware to see workloads and their example references.
Each shuffled round visits every eligible workload once. Queues are drained,
outputs must arrive, stop()/wait() must complete, and the connection must survive.
Models use the same Model Zoo slugs as the examples (downloads may be needed).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from datetime import timedelta
from functools import partial
import json
import math
import os
from pathlib import Path
import random
import re
import tempfile
import time
import traceback
from typing import Any, Callable


@dataclass
class Case:
    name: str
    build: Callable
    nodes: tuple[str, ...]
    source: str
    needs: tuple[str, ...] = ()
    depends: tuple[str, ...] = ()


def node_class(dai, name):
    namespace = dai
    for part in ("beta.node." + name[5:] if name.startswith("beta.") else "node." + name).split("."):
        namespace = getattr(namespace, part)
    return namespace


@dataclass
class Workload:
    dai: Any
    device: Any
    args: Any
    state: dict
    case: Case
    pipeline: Any = field(init=False)
    streams: list = field(default_factory=list)
    start_actions: list = field(default_factory=list)
    callbacks: list = field(default_factory=list)
    cleanup: list = field(default_factory=list)
    created: set = field(default_factory=set)
    metrics: dict = field(default_factory=dict)

    def __post_init__(self):
        self.pipeline = self.dai.Pipeline(self.device)

    def create(self, name):
        self.created.add(name)
        return self.pipeline.create(node_class(self.dai, name))

    def watch(self, output, name=None, check=None):
        queue = output.createOutputQueue(maxSize=4, blocking=False)
        self.streams.append([name or f"output_{len(self.streams)}", queue, 0, check])

    def camera_node(self, socket=None):
        if socket is None:
            socket = self.state["camera_sockets"][0]
        return self.create("Camera").build(socket, sensorFps=self.args.fps)

    def camera(self, socket=None, frame_type=None, full=False):
        dai = self.dai
        cam = self.camera_node(socket)
        if full:
            output = cam.requestFullResolutionOutput(fps=self.args.fps)
        else:
            output = cam.requestOutput(
                (640, 400), type=frame_type or dai.ImgFrame.Type.NV12, fps=self.args.fps
            )
        return cam, output

    def stereo_inputs(self, full=False):
        left, right = self.state["stereo_pair"]
        return self.camera(left, full=full), self.camera(right, full=full)

    def stereo(self):
        (_, left), (_, right) = self.stereo_inputs()
        stereo = self.create("StereoDepth")
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        left.link(stereo.left)
        right.link(stereo.right)
        return stereo

    def depth(self, algorithm="AUTO"):
        depth = self.create("Depth").build(
            getattr(self.dai.node.Depth.Algorithm, algorithm), self.args.fps, (640, 400)
        )
        return depth

    def imu(self):
        imu = self.create("IMU")
        imu.enableIMUSensor(
            [self.dai.IMUSensor.ACCELEROMETER_RAW, self.dai.IMUSensor.GYROSCOPE_RAW], 200
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        return imu


def camera_pipeline(run):
    for socket in run.state["camera_sockets"]:
        _, output = run.camera(socket)
        previous = [None]

        def check(frame, previous=previous):
            stamp = frame.getTimestampDevice().total_seconds()
            if previous[0] is not None and stamp <= previous[0]:
                raise AssertionError("Camera timestamps did not advance")
            previous[0] = stamp

        run.watch(output, str(socket), check)


def imu_pipeline(run):
    def check(message):
        if not message.packets:
            raise AssertionError("IMU returned an empty packet batch")
        for sensor in ("acceleroMeter", "gyroscope"):
            stamp = getattr(message.packets[-1], sensor).getTimestampDevice().total_seconds()
            key = "last_imu_timestamp_" + sensor
            previous = run.metrics.get(key)
            if previous is not None and stamp <= previous:
                raise AssertionError(f"IMU {sensor} timestamps did not advance")
            run.metrics[key] = stamp
        run.metrics["imu_packets"] = run.metrics.get("imu_packets", 0) + len(message.packets)

    run.watch(run.imu().out, "imu", check)


def cameras_imu_pipeline(run):
    camera_pipeline(run)
    imu_pipeline(run)


def fsync_pipeline(run, fps_scale=1, with_imu=False):
    dai, device = run.dai, run.device
    original_role = device.getExternalFrameSyncRole()
    # Restore the role after stopping, including failed starts and Ctrl+C.
    def restore():
        device.setExternalStrobeEnable(False)
        ok, reason = device.setExternalFrameSyncRole(original_role)
        if not ok:
            raise RuntimeError(f"Restoring frame-sync role failed: {reason}")

    run.cleanup.append(restore)
    role = getattr(dai.ExternalFrameSyncRole, run.args.fsync_role)
    ok, reason = device.setExternalFrameSyncRole(role)
    if not ok:
        raise RuntimeError(f"Setting frame-sync role failed: {reason}")
    if device.getExternalFrameSyncRole() != role:
        raise AssertionError("Frame-sync controller did not retain the requested role")
    if role == dai.ExternalFrameSyncRole.MASTER:
        device.setExternalStrobeRelativeLimits(0.005, 0.995)
        device.setExternalStrobeEnable(run.state["fsync_sockets"][0])

    fps = run.args.fps * fps_scale
    # Pair within half a frame first, then test the stricter tolerance ourselves.
    # A tight Sync threshold alone could conceal bad synchronization by emitting nothing.
    sync = run.create("Sync")
    sync.setRunOnHost(True)
    sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
    sockets = run.state["fsync_sockets"]
    for index, socket in enumerate(sockets):
        camera = run.create("Camera")
        if role == dai.ExternalFrameSyncRole.MASTER:
            camera.build(socket, sensorFps=fps)
        else:
            camera.build(socket)
        camera.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
        camera.initialControl.setManualExposure(1000, 100)
        output = camera.requestOutput((640, 400), type=dai.ImgFrame.Type.NV12, fps=fps)
        output.link(sync.inputs[str(index)])

    last_timestamps = {}
    samples = []
    first_group = [None]

    def check(group):
        timestamps = []
        for index in range(len(sockets)):
            frame = group[str(index)]
            if frame.getFsync() != dai.ImgFrame.Fsync.INPUT:
                raise AssertionError(f"{sockets[index]} is not locked to FSYNC INPUT")
            stamp = frame.getTimestamp(dai.CameraExposureOffset.END).total_seconds()
            if index in last_timestamps:
                period = stamp - last_timestamps[index]
                if period <= 0:
                    raise AssertionError("Camera timestamps did not advance")
                run.metrics.setdefault("frame_periods_s", []).append(period)
            last_timestamps[index] = stamp
            timestamps.append(stamp)
        now = time.monotonic()
        if first_group[0] is None:
            first_group[0] = now
        if now - first_group[0] >= run.args.fsync_settle:
            skew_ms = (max(timestamps) - min(timestamps)) * 1000
            samples.append(skew_ms)
            run.metrics.update(fsync_groups=len(samples), max_skew_ms=max(samples), requested_fps=fps)

    def validate():
        if len(samples) < run.args.fsync_min_samples:
            raise AssertionError(f"Only {len(samples)} settled FSYNC groups received")
        mean_skew_ms = sum(samples) / len(samples)
        p99_skew_ms = sorted(samples)[math.ceil(0.99 * len(samples)) - 1]
        run.metrics.update(mean_skew_ms=mean_skew_ms, p99_skew_ms=p99_skew_ms)
        # Use the same acceptance limits as tests/src/onhost_tests/multi_device_fsync_test.cpp.
        if mean_skew_ms > run.args.fsync_mean_tolerance_ms:
            raise AssertionError(f"FSYNC mean skew {mean_skew_ms:.3f} ms exceeds tolerance")
        if p99_skew_ms > run.args.fsync_tolerance_ms:
            raise AssertionError(f"FSYNC P99 skew {p99_skew_ms:.3f} ms exceeds tolerance")
        periods = sorted(run.metrics.pop("frame_periods_s", []))
        if periods:
            measured_fps = 1 / periods[len(periods) // 2]
            run.metrics["measured_fps"] = measured_fps
            if abs(measured_fps - fps) / fps > 0.15:
                raise AssertionError(f"FSYNC rate {measured_fps:.2f} differs from requested {fps:.2f}")
        run.metrics.update(fsync_groups=len(samples), max_skew_ms=max(samples), requested_fps=fps)

    run.watch(sync.out, "fsync", check)
    run.callbacks.append(validate)
    if with_imu:
        imu_pipeline(run)


def tof_pipeline(run):
    tof = run.create("ToF").build(
        boardSocket=run.state["tof_sockets"][0],
        profile=run.dai.ToFConfig.Profile.MID_RANGE,
        fps=run.args.fps,
    )
    run.watch(tof.depth, "tof_depth")


def tof_confidence_pipeline(run):
    tof = run.create("ToF").build(boardSocket=run.state["tof_sockets"][0], fps=run.args.fps)
    confidence = run.create("ToFDepthConfidenceFilter").build(tof.depth, tof.amplitude)
    confidence.setRunOnHost(True)
    run.watch(confidence.filteredDepth, "filtered_depth")


def depth_pipeline(run, algorithm):
    run.watch(run.depth(algorithm).depth, "depth")


def stereo_pipeline(run, kind="StereoDepth"):
    if kind in ("StereoDepth", "ImageFilters", "DynamicCalibration", "AutoCalibration"):
        stereo = run.stereo()
        if kind == "ImageFilters":
            filters = run.create(kind)
            filters.setRunOnHost(True)
            filters.build(stereo.disparity)
            filters.initialConfig.filterIndices = []
            filters.initialConfig.filterParams = [run.dai.node.ImageFilters.MedianFilterParams.KERNEL_3x3]
            run.watch(filters.output, "filtered_disparity")
        elif kind == "DynamicCalibration":
            calibration = run.create(kind)
            stereo.syncedLeft.link(calibration.left)
            stereo.syncedRight.link(calibration.right)
            run.watch(stereo.depth, "depth")
        elif kind == "AutoCalibration":
            cameras = [n for n in run.pipeline.getAllNodes() if isinstance(n, run.dai.node.Camera)]
            calibration = run.create(kind).build(*cameras)
            calibration.initialConfig.flashCalibration = False
            calibration.initialConfig.mode = run.dai.AutoCalibrationConfig.ON_START
            # The worker can apply RAM calibration; restore it between workloads.
            original = run.device.getCalibration()
            run.cleanup.append(lambda: run.device.setCalibration(original))
            run.watch(stereo.depth, "depth")
        else:
            run.watch(stereo.depth, "depth")
        return
    (_, left), (_, right) = run.stereo_inputs(full=True)
    if kind == "Rectification":
        rectification = run.create(kind)
        left.link(rectification.input1)
        right.link(rectification.input2)
        run.watch(rectification.output1, "left")
        run.watch(rectification.output2, "right")
    elif kind in ("NeuralDepth", "NeuralAssistedStereo"):
        model = run.dai.DeviceModelZoo.NEURAL_DEPTH_NANO
        node = run.create(kind).build(left, right, model)
        run.watch(node.depth, "depth")
    elif kind == "GPUStereo":
        node = run.create(kind)
        left.link(node.left)
        right.link(node.right)
        run.watch(node.disparity, "disparity")


def image_pipeline(run, kind):
    dai = run.dai
    _, output = run.camera(frame_type=dai.ImgFrame.Type.NV12)
    node = run.create(kind)
    if kind == "ImageManip":
        node.initialConfig.setOutputSize(320, 240)
        node.initialConfig.addFlipHorizontal()
        node.setMaxOutputFrameSize(320 * 240 * 3)
        output.link(node.inputImage)
        run.watch(node.out)
    elif kind == "Warp":
        node.setInterpolation(dai.Interpolation.BILINEAR)
        node.setWarpMesh([dai.Point2f(0, 0), dai.Point2f(640, 0),
                          dai.Point2f(0, 400), dai.Point2f(640, 400)], 2, 2)
        node.setOutputSize((640, 400))
        node.setMaxOutputFrameSize(640 * 400 * 3)
        output.link(node.inputImage)
        run.watch(node.out)
    elif kind == "AprilTag":
        output.link(node.inputImage)
        run.watch(node.out)
    elif kind == "FeatureTracker":
        manip = run.create("ImageManip")
        # RVC4 feature extraction requires width/height multiples of 128/32.
        manip.initialConfig.setOutputSize(640, 384)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)
        output.link(manip.inputImage)
        manip.out.link(node.inputImage)
        run.watch(node.outputFeatures)
    elif kind == "Gate":
        output.link(node.input)
        control = node.inputControl.createInputQueue()
        run.start_actions.append(lambda: control.send(dai.GateControl.openGate()))
        run.watch(node.output)
    elif kind == "Script":
        output.link(node.inputs["in"])
        node.setScript("while True:\n    node.io['out'].send(node.io['in'].get())\n")
        run.watch(node.outputs["out"])
    elif kind == "VideoEncoder":
        node.build(output, frameRate=run.args.fps, profile=dai.VideoEncoderProperties.Profile.H264_MAIN)
        run.watch(node.out)


def sync_pipeline(run):
    _, output = run.camera()
    sync = run.create("Sync")
    sync.setRunOnHost(True)
    output.link(sync.inputs["image"])
    demux = run.create("MessageDemux")
    demux.setRunOnHost(True)
    sync.out.link(demux.input)
    run.watch(demux.outputs["image"], "image")


def benchmark_pipeline(run):
    generator = run.create("BenchmarkOut")
    generator.setRunOnHost(True)
    generator.setFps(run.args.fps)
    sink = run.create("BenchmarkIn")
    sink.setRunOnHost(False)
    sink.sendReportEveryNMessages(5)
    sink.logReportsAsWarnings(False)
    generator.out.link(sink.input)
    queue = generator.input.createInputQueue()
    run.start_actions.append(lambda: queue.send(run.dai.ImgFrame()))
    run.watch(sink.report)


def logger_pipeline(run):
    logger = run.create("SystemLogger")
    logger.setRate(5)
    run.watch(logger.out)


def aligned_depth_pipeline(run, pointcloud=False):
    dai = run.dai
    _, color = run.camera(run.state["color_socket"], dai.ImgFrame.Type.RGB888i)
    depth = run.depth()
    align = run.create("ImageAlign")
    depth.depth.link(align.input)
    color.link(align.inputAlignTo)
    if pointcloud:
        pc = run.create("PointCloud")
        align.outputAligned.link(pc.inputDepth)
        color.link(pc.inputColor)
        run.watch(pc.outputPointCloud)
    else:
        run.watch(align.outputAligned)


def rgbd_pipeline(run):
    color = run.camera_node(run.state["color_socket"])
    rgbd = run.create("RGBD").build(color, run.depth(), (640, 400), run.args.fps)
    run.watch(rgbd.pcl)


def spatial_pipeline(run):
    dai = run.dai
    depth = run.depth()
    spatial = run.create("SpatialLocationCalculator")
    config = dai.SpatialLocationCalculatorConfigData()
    config.depthThresholds.lowerThreshold = 100
    config.depthThresholds.upperThreshold = 10000
    config.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
    spatial.initialConfig.addROI(config)
    spatial.inputConfig.setWaitForMessage(False)
    depth.depth.link(spatial.inputDepth)
    run.watch(spatial.out)


def inference_pipeline(run, kind):
    dai = run.dai
    camera = run.camera_node()
    if kind == "SegmentationParser":
        model = "luxonis/deeplab-v3-plus:512x512"
        nn = run.create("NeuralNetwork").build(camera, model)
        parser = run.create(kind).build(nn.out, model)
        run.watch(parser.out)
    elif kind == "NeuralNetwork":
        nn = run.create(kind).build(camera, dai.NNModelDescription("yolov6-nano"))
        run.watch(nn.out)
    elif kind == "SpatialDetectionNetwork":
        nn = run.create(kind).build(camera, run.depth(), "yolov6-nano")
        run.watch(nn.out)
    else:
        nn = run.create("DetectionNetwork").build(camera, dai.NNModelDescription("yolov6-nano"))
        if kind == "ObjectTracker":
            tracker = run.create(kind)
            tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
            tracker.setDetectionLabelsToTrack([0])
            nn.passthrough.link(tracker.inputTrackerFrame)
            nn.passthrough.link(tracker.inputDetectionFrame)
            nn.out.link(tracker.inputDetections)
            run.watch(tracker.out)
        elif kind == "Align":
            align = run.create(kind)
            nn.out.link(align.input)
            camera.requestOutput((800, 600), fps=run.args.fps).link(align.inputAlignTo)
            run.watch(align.outputAligned)
        else:
            run.watch(nn.out)


def beta_parser_pipeline(run, kind, model):
    camera = run.camera_node()
    archive = run.dai.NNArchive(run.dai.getModelFromZoo(run.dai.NNModelDescription(
        model, platform=run.device.getPlatformAsString()
    )))
    nn = run.create("NeuralNetwork").build(camera, archive)
    parser = run.create("beta." + kind).build(nn.out, archive)
    run.watch(parser.out)


def detections_filter_pipeline(run):
    node = run.create("beta.ImgDetectionsFilter")
    queue = node.input.createInputQueue()
    run.start_actions.append(lambda: queue.send(run.dai.ImgDetections()))
    run.watch(node.output)


def fusion_pipeline(run):
    left, right = (run.camera_node(socket) for socket in run.state["stereo_pair"])
    fusion = run.create("beta.ToFStereoFusion").build(left, right, run.args.fps)
    run.watch(fusion.depth)


def stitching_pipeline(run):
    (_, left), (_, right) = run.stereo_inputs()
    stitching = run.create("Stitching").build([left, right])
    stitching.setMode(run.dai.node.Stitching.Mode.PLANAR_PROJECTION)
    stitching.setPlane(run.dai.Point3f(0, 0, 130), run.dai.Point3f(0, 0, 1), run.dai.LengthUnit.CENTIMETER)
    stitching.setMaxViewSize(1280, 720)
    run.watch(stitching.out)


def host_nodes_pipeline(run):
    dai = run.dai

    class Forward(dai.node.HostNode):
        def __init__(self):
            super().__init__()
            self.output = self.createOutput()

        def process(self, frame):
            self.output.send(frame)

    class ThreadedForward(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.input = self.createInput()
            self.output = self.createOutput()

        def run(self):
            while self.mainLoop():
                try:
                    message = self.input.get()
                except dai.MessageQueue.QueueException:
                    return
                self.output.send(message)

    _, output = run.camera()
    forward = run.pipeline.create(Forward)
    forward.link_args(output)
    threaded = run.pipeline.create(ThreadedForward)
    forward.output.link(threaded.input)
    run.created.update(("HostNode", "ThreadedHostNode"))
    run.watch(threaded.output)


def recording_pipeline(run, metadata=False, replay=False):
    if metadata:
        path = run.state["output_dir"] / "imu.mcap"
        if replay:
            node = run.create("ReplayMetadataOnly")
            node.setReplayFile(path)
            node.setLoop(True)
            run.watch(node.out)
        else:
            imu = run.imu()
            node = run.create("RecordMetadataOnly")
            node.setRecordFile(path)
            imu.out.link(node.input)
            run.watch(imu.out)
    else:
        path = run.state["output_dir"] / "camera.mp4"
        if replay:
            node = run.create("ReplayVideo")
            node.setReplayVideoFile(path)
            node.setLoop(True)
            run.watch(node.out)
        else:
            _, output = run.camera()
            encoder = run.create("VideoEncoder").build(
                output, frameRate=run.args.fps, profile=run.dai.VideoEncoderProperties.Profile.H264_MAIN
            )
            node = run.create("RecordVideo")
            node.setFps(round(run.args.fps))
            node.setRecordVideoFile(path)
            encoder.out.link(node.input)
            run.watch(encoder.out)


def slam_pipeline(run, kind):
    stereo = run.stereo()
    imu = run.imu()
    odom = run.create(kind)
    imu.out.link(odom.imu)
    if kind == "BasaltVIO":
        stereo.syncedLeft.link(odom.left)
        stereo.syncedRight.link(odom.right)
        slam = run.create("RTABMapSLAM")
        stereo.depth.link(slam.depth)
        stereo.rectifiedLeft.link(slam.rect)
        odom.transform.link(slam.odom)
        run.watch(slam.transform)
    else:
        stereo.rectifiedLeft.link(odom.rect)
        stereo.depth.link(odom.depth)
        odom.setUseFeatures(False)
        run.watch(odom.transform)


def cases():
    result = [
        Case("cameras", camera_pipeline, ("Camera",), "Camera/camera_all.py", ("camera",)),
        Case("imu", imu_pipeline, ("IMU",), "IMU/imu_gyroscope_accelerometer.py", ("imu",)),
        Case("cameras_imu", cameras_imu_pipeline, ("Camera", "IMU"), "Camera/camera_all.py", ("camera", "imu")),
        Case("fsync", fsync_pipeline, ("Camera", "Sync"), "MultiDevice/multi_device_frame_sync.py", ("fsync",)),
        Case("fsync_half_rate", partial(fsync_pipeline, fps_scale=0.5), ("Camera", "Sync"), "MultiDevice/multi_device_frame_sync.py", ("fsync",)),
        Case("fsync_imu", partial(fsync_pipeline, with_imu=True), ("Camera", "Sync", "IMU"), "MultiDevice/multi_device_frame_sync.py", ("fsync", "imu")),
        Case("tof", tof_pipeline, ("ToF", "ToFBase"), "ToF/tof_minimal.py", ("tof",)),
        Case("tof_confidence", tof_confidence_pipeline, ("ToF", "ToFDepthConfidenceFilter"), "ToF/tof_all_queues.py", ("tof",)),
        Case("sync_demux", sync_pipeline, ("Camera", "Sync", "MessageDemux"), "Sync", ("camera",)),
        Case("benchmark", benchmark_pipeline, ("BenchmarkIn", "BenchmarkOut"), "Benchmark/benchmark_simple.py"),
        Case("logger", logger_pipeline, ("SystemLogger",), "Misc/SystemLogger/system_logger.py"),
        Case("image_align", aligned_depth_pipeline, ("Camera", "Depth", "ImageAlign"), "ImageAlign/depth_align.py", ("color", "depth")),
        Case("pointcloud", partial(aligned_depth_pipeline, pointcloud=True), ("Camera", "Depth", "ImageAlign", "PointCloud"), "ToF/tof_pointcloud.py", ("color", "depth")),
        Case("rgbd", rgbd_pipeline, ("Camera", "Depth", "RGBD"), "RGBD/rgbd.py", ("color", "depth")),
        Case("spatial_location", spatial_pipeline, ("Depth", "SpatialLocationCalculator"), "SpatialLocationCalculator/spatial_location_calculator.py", ("depth",)),
        Case("stitching", stitching_pipeline, ("Camera", "Stitching", "Sync"), "MultiDevice/multi_device_stitching_planar_projection.py", ("stereo",)),
        Case("host_nodes", host_nodes_pipeline, ("Camera", "HostNode", "ThreadedHostNode"), "HostNodes", ("camera",)),
        Case("record_video", recording_pipeline, ("Camera", "VideoEncoder", "RecordVideo"), "RecordReplay/record_video.py", ("camera",)),
        Case("replay_video", partial(recording_pipeline, replay=True), ("ReplayVideo",), "RecordReplay/replay_video.py", depends=("record_video",)),
        Case("record_imu", partial(recording_pipeline, metadata=True), ("IMU", "RecordMetadataOnly"), "../cpp/RecordReplay/record_imu.cpp", ("imu",)),
        Case("replay_imu", partial(recording_pipeline, metadata=True, replay=True), ("ReplayMetadataOnly",), "../cpp/RecordReplay/replay_imu.cpp", ("imu",), ("record_imu",)),
        Case("basalt_slam", partial(slam_pipeline, kind="BasaltVIO"), ("Camera", "StereoDepth", "IMU", "BasaltVIO", "RTABMapSLAM"), "VSLAM/basalt_vio_rtabmap_slam.py", ("stereo", "imu")),
        Case("rtabmap_vio", partial(slam_pipeline, kind="RTABMapVIO"), ("Camera", "StereoDepth", "IMU", "RTABMapVIO"), "VSLAM", ("stereo", "imu")),
        Case("detections_filter", detections_filter_pipeline, ("beta.ImgDetectionsFilter",), "Beta/img_detections_filter.py"),
        Case("tof_stereo_fusion", fusion_pipeline, ("Camera", "beta.ToFStereoFusion"), "Beta/tof_neural_fusion.py", ("stereo", "tof", "neural")),
    ]
    for kind, source in {
        "ImageManip": "ImageManip/image_manip_multi_ops.py", "Warp": "Warp/warp_mesh.py",
        "AprilTag": "AprilTags/april_tags.py", "FeatureTracker": "FeatureTracker/feature_tracker.py",
        "Gate": "Gate/gate_example.py", "Script": "Script/script_switch_all_cameras.py",
        "VideoEncoder": "VideoEncoder/video_encode.py",
    }.items():
        result.append(Case(kind, partial(image_pipeline, kind=kind), ("Camera", kind), source, ("camera",)))
    for kind, source in {
        "StereoDepth": "StereoDepth/stereo.py", "ImageFilters": "StereoDepth/stereo_depth_filters.py",
        "Rectification": "Vpp/virtual_patern_projection.py", "GPUStereo": "RVC4/GPUStereo/gpu_stereo.py",
        "NeuralDepth": "NeuralDepth/neural_depth.py", "NeuralAssistedStereo": "NeuralAssistedStereo/neural_assisted_stereo.py",
        "DynamicCalibration": "DynamicCalibration/calibration_dynamic.py", "AutoCalibration": "AutoCalibration/auto_calibration_example.py",
    }.items():
        nodes = ("Camera", kind)
        needs = ("stereo",)
        if kind in ("NeuralDepth", "NeuralAssistedStereo"):
            needs += ("neural",)
        if kind == "NeuralAssistedStereo":
            nodes += ("Rectification", "NeuralDepth", "Vpp", "StereoDepth")
        if kind == "GPUStereo":
            needs += ("gpu",)
        result.append(Case(kind, partial(stereo_pipeline, kind=kind), nodes, source, needs))
    for algorithm in ("AUTO", "STEREO", "TOF", "NEURAL", "NEURAL_ASSISTED_STEREO", "GPU_STEREO", "TOF_STEREO_FUSION"):
        result.append(Case("depth_" + algorithm.lower(), partial(depth_pipeline, algorithm=algorithm),
                           ("Depth",), "Depth/unified_depth.py", ("algorithm:" + algorithm,)))
    for kind, source in {
        "NeuralNetwork": "NeuralNetwork/neural_network.py", "DetectionNetwork": "DetectionNetwork/detection_network.py",
        "ObjectTracker": "ObjectTracker/object_tracker.py", "SpatialDetectionNetwork": "SpatialDetectionNetwork/spatial_detection.py",
        "SegmentationParser": "Segmentation/semantic_segmentation.py", "Align": "Align/img_detections_align.py",
    }.items():
        nodes = ("Camera", kind, "NeuralNetwork")
        if kind in ("DetectionNetwork", "ObjectTracker", "Align"):
            nodes += ("DetectionNetwork", "DetectionParser")
        elif kind == "SpatialDetectionNetwork":
            nodes += ("DetectionParser",)
        needs = ("camera", "depth") if kind == "SpatialDetectionNetwork" else ("camera",)
        result.append(Case(kind, partial(inference_pipeline, kind=kind), nodes, source, needs))
    for kind, slug, source in BETA_MODELS:
        result.append(Case(kind, partial(beta_parser_pipeline, kind=kind, model=slug),
                           ("Camera", "NeuralNetwork", "beta." + kind), "Beta/" + source, ("camera",)))
    return result


BETA_MODELS = [
    ("ClassificationParser", "luxonis/emotion-recognition:260x260", "classification_parser.py"),
    ("ClassificationSequenceParser", "luxonis/paddle-text-recognition:320x48", "classification_sequence_parser.py"),
    ("EmbeddingsParser", "luxonis/arcface:lfw-112x112", "embeddings_parser.py"),
    ("FastSAMParser", "luxonis/fastsam-s:512x288", "fastsam_parser.py"),
    ("HRNetParser", "luxonis/lite-hrnet:18-coco-288x384", "hrnet_parser.py"),
    ("ImageOutputParser", "luxonis/dncnn3:320x240", "image_output_parser.py"),
    ("KeypointParser", "luxonis/mediapipe-face-landmarker:192x192", "keypoint_parser.py"),
    ("LaneDetectionParser", "luxonis/ultra-fast-lane-detection:culane-800x288", "lane_detection_parser.py"),
    ("MapOutputParser", "luxonis/dm-count:sha-426x240", "map_output_parser.py"),
    ("MLSDParser", "luxonis/m-lsd:512x512", "mlsd_parser.py"),
    ("MPPalmDetectionParser", "luxonis/mediapipe-palm-detection:192x192", "mp_palm_detection_parser.py"),
    ("PPTextDetectionParser", "luxonis/paddle-text-detection:256x256", "pp_text_detection_parser.py"),
    ("SCRFDParser", "luxonis/scrfd-face-detection:10g-640x640", "scrfd_parser.py"),
    ("SuperAnimalParser", "luxonis/superanimal-landmarker:256x256", "superanimal_parser.py"),
    ("YuNetParser", "luxonis/yunet:640x480", "yunet_parser.py"),
]


def example_inventory():
    root = Path(__file__).resolve().parents[1] / "examples"
    nodes = {}
    for path in sorted(root.rglob("*")):
        if path.suffix not in (".py", ".cpp", ".hpp"):
            continue
        source = path.read_text()
        patterns = [r"dai\.(beta\.)?node\.(\w+)", r"dai::(beta::)?node::(\w+)"]
        for pattern in patterns:
            for beta, name in re.findall(pattern, source):
                if name == "internal":
                    continue
                key = ("beta." if beta else "") + name
                nodes.setdefault(key, []).append(str(path.relative_to(root)))
    return {key: sorted(set(paths)) for key, paths in sorted(nodes.items())}


# These example names cannot be exercised by a single reusable RVC4 connection.
EXCLUDED = {
    "beta.MultiDeviceCalibration": "Requires multiple physical devices and cross-device calibration targets",
    "EdgeDetector": "RVC2-only node; RVC2 lacks stop-without-close firmware support",
    "Thermal": "RVC2 thermal camera example; RVC2 lacks stop-without-close support",
    "CustomNode": "C++ example-local class; represented by the Python HostNode workload",
    "CustomThreadedNode": "C++ example-local class; represented by ThreadedHostNode",
    "Display": "C++ example-local display class; represented by HostNode without GUI",
    "HostCamera": "C++ example-local host camera, requires an additional host video device",
    "DepthSource": "C++ example-local helper, not an SDK node",
}


def inspect_device(dai, device, output_dir):
    features = device.getConnectedCameraFeatures()
    sockets = [f.socket for f in features if any(t in f.supportedTypes for t in
               (dai.CameraSensorType.COLOR, dai.CameraSensorType.MONO))]
    colors = [f.socket for f in features if dai.CameraSensorType.COLOR in f.supportedTypes]
    tof = [f.socket for f in features if dai.CameraSensorType.TOF in f.supportedTypes]
    pairs = device.getStereoPairs()
    pair = (pairs[0].left, pairs[0].right) if pairs else ()
    neural = device.isNeuralDepthSupported()
    gpu = device.isGpuStereoSupported()
    is_rvc4 = device.getPlatform() == dai.Platform.RVC4
    methods = {"STEREO"} if pair else set()
    if pair and is_rvc4 and neural:
        if device.getSupportedDeviceModels():
            methods.add("NEURAL")
        methods.add("NEURAL_ASSISTED_STEREO")
    if pair and is_rvc4 and gpu:
        methods.add("GPU_STEREO")
    if tof:
        methods.add("TOF")
        if pair and is_rvc4:
            methods.add("TOF_STEREO_FUSION")
    return dict(camera_sockets=sockets, color_socket=colors[0] if colors else None,
                tof_sockets=tof, stereo_pair=pair, fsync_sockets=list(sockets),
                imu=device.getConnectedIMU().upper() not in ("", "NONE"), neural=neural,
                gpu=gpu, algorithms=methods, output_dir=output_dir)


def unsupported_reason(case, dai, state):
    for name in case.nodes:
        try:
            node_class(dai, name)
        except AttributeError:
            return f"{name} is absent from this DepthAI build"
    for requirement in case.needs:
        ready = {"camera": bool(state["camera_sockets"]), "color": state["color_socket"] is not None,
                 "stereo": bool(state["stereo_pair"]), "imu": state["imu"],
                 "tof": bool(state["tof_sockets"]), "depth": bool(state["algorithms"]),
                 "neural": state["neural"], "gpu": state["gpu"],
                 "fsync": len(state["fsync_sockets"]) >= 2}
        if requirement.startswith("algorithm:"):
            algorithm = requirement.split(":", 1)[1]
            if algorithm != "AUTO" and algorithm not in state["algorithms"]:
                return f"Depth algorithm {algorithm} is unsupported"
            if not state["algorithms"]:
                return "No depth algorithms available"
        elif not ready[requirement]:
            return f"Missing capability: {requirement}"
    return None


def next_case(pending, completed, rng):
    ready = [case for case in pending if set(case.depends) <= completed]
    if not ready:
        raise RuntimeError("No workloads can run: missing or cyclic recording dependencies")
    chosen = rng.choice(ready)
    pending.remove(chosen)
    return chosen


def run_random_pipeline(device, case, args, state, dai):
    run = Workload(dai, device, args, state, case)
    device_id = device.getDeviceId()
    record = {"case": case.name, "status": "failed", "nodes": [], "outputs": {}, "metrics": {}}
    started = time.monotonic()
    try:
        case.build(run)
        run.pipeline.start()
        for node in run.pipeline.getAllNodes():
            name = type(node).__name__
            if type(node).__module__.startswith("depthai.beta"):
                name = "beta." + name
            run.created.add(name)
        absent = set(case.nodes) - run.created
        if absent:
            raise AssertionError(f"Workload did not create its declared nodes: {sorted(absent)}")
        for action in run.start_actions:
            action()
        deadline = time.monotonic() + args.runtime
        while time.monotonic() < deadline:
            if not run.pipeline.isRunning():
                raise RuntimeError("Pipeline stopped before its requested runtime")
            for stream in run.streams:
                _, queue, _, check = stream
                # A bounded drain keeps a fast stream from starving other streams.
                for _ in range(4):
                    message = queue.tryGet()
                    if message is None:
                        break
                    stream[2] += 1
                    if check:
                        check(message)
            time.sleep(min(0.005, max(0, deadline - time.monotonic())))
        missing = [name for name, _, count, _ in run.streams if count == 0]
        if missing:
            raise AssertionError(f"No output received: {', '.join(missing)}")
        for validate in run.callbacks:
            validate()
        record["status"] = "passed"
    except BaseException:
        record["error"] = traceback.format_exc()
        raise
    finally:
        try:
            try:
                try:
                    run.pipeline.stop()
                finally:
                    run.pipeline.wait()
            finally:
                try:
                    for cleanup in reversed(run.cleanup):
                        cleanup()
                finally:
                    run.metrics.pop("frame_periods_s", None)
            if device.isClosed():
                raise AssertionError("Pipeline closed the caller-owned device")
            if device.getDeviceId() != device_id:
                raise AssertionError("Device identity changed between pipelines")
            if device.isPipelineRunning():
                raise AssertionError("Device pipeline is still running after stop()/wait()")
        except BaseException:
            record["status"] = "failed"
            record["teardown_error"] = traceback.format_exc()
            raise
        finally:
            record["seconds"] = time.monotonic() - started
            record["nodes"] = sorted(run.created)
            record["outputs"] = {name: count for name, _, count, _ in run.streams}
            record["metrics"] = run.metrics
            state["runs"].append(record)
            if "output_dir" in state:
                with (state["output_dir"] / "runs.jsonl").open("a") as log:
                    log.write(json.dumps(record) + "\n")
            print(json.dumps(record), flush=True)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--device", help="Device ID or IP; defaults to the first available device")
    result.add_argument("--runtime", type=float, default=5, help="Seconds per pipeline after start (default: 5)")
    result.add_argument("--duration", type=float, default=600, help="Total scheduling budget in seconds (default: 600)")
    result.add_argument("--fps", type=float, default=15)
    result.add_argument("--seed", type=int, default=0)
    result.add_argument("--case", action="append", dest="selected", help="Run only this workload; repeatable")
    result.add_argument("--output-dir", type=Path, help="Parent for a fresh results/recordings directory")
    result.add_argument("--list", action="store_true", help="List workload/node/example coverage without a device")
    result.add_argument("--fsync-role", choices=("MASTER", "SLAVE"), default="MASTER",
                        help="SLAVE requires an external FSYNC signal at --fps")
    result.add_argument("--fsync-tolerance-ms", type=float, default=2, help="Maximum P99 frame-sync skew in ms")
    result.add_argument("--fsync-mean-tolerance-ms", type=float, default=1, help="Maximum mean frame-sync skew in ms")
    result.add_argument("--fsync-settle", type=float, default=1, help="Seconds of frame-sync warmup")
    result.add_argument("--fsync-min-samples", type=int, default=5)
    return result


def main():
    cli = parser()
    args = cli.parse_args()
    for name in ("runtime", "duration", "fps", "fsync_tolerance_ms", "fsync_mean_tolerance_ms", "fsync_min_samples"):
        if getattr(args, name) <= 0:
            cli.error(f"--{name.replace('_', '-')} must be positive")
    if args.fsync_settle < 0:
        cli.error("--fsync-settle must be nonnegative")
    all_cases = cases()
    inventory = example_inventory()
    covered = {name for case in all_cases for name in case.nodes}
    unhandled = set(inventory) - covered - EXCLUDED.keys()
    if unhandled:
        cli.error(f"Example nodes have no workload or exclusion: {sorted(unhandled)}")
    if args.list:
        print(json.dumps({"cases": [{"name": c.name, "nodes": c.nodes, "source": c.source,
                                    "needs": c.needs, "depends": c.depends} for c in all_cases],
                          "example_nodes": inventory, "excluded": EXCLUDED}, indent=2))
        return 0
    if args.selected:
        wanted = set(args.selected)
        unknown = wanted - {c.name for c in all_cases}
        if unknown:
            cli.error(f"Unknown workloads: {sorted(unknown)}; see --list")
        # Replay fixtures are produced on this same connection before replaying them.
        for case in all_cases:
            if case.name in wanted:
                wanted.update(case.depends)
        all_cases = [case for case in all_cases if case.name in wanted]

    import depthai as dai

    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    output_dir = Path(tempfile.mkdtemp(prefix="pipeline-restart-", dir=args.output_dir))
    report = {"seed": args.seed, "runs": [], "skipped": {}, "excluded": EXCLUDED,
              "example_nodes": inventory, "sdk": dai.__file__,
              "firmware": os.environ.get("DEPTHAI_DEVICE_RVC4_FWP", "SDK default")}
    print(f"SDK: {report['sdk']}", flush=True)
    print(f"Firmware: {report['firmware']}", flush=True)
    print(f"Results: {output_dir}", flush=True)
    try:
        with dai.Device(dai.DeviceInfo(args.device)) if args.device else dai.Device() as device:
            if device.getPlatform() != dai.Platform.RVC4:
                raise RuntimeError("Reusing one connection requires RVC4 pipeline-stop firmware support")
            device.setMaxReconnectionAttempts(0)
            state = inspect_device(dai, device, output_dir)
            state["runs"] = report["runs"]
            report["device"] = device.getDeviceId()
            eligible = []
            for case in all_cases:
                reason = ("An external SLAVE clock cannot be changed by this test"
                          if case.name == "fsync_half_rate" and args.fsync_role == "SLAVE"
                          else unsupported_reason(case, dai, state))
                if reason:
                    report["skipped"][case.name] = reason
                    print(f"SKIP {case.name}: {reason}", flush=True)
                else:
                    eligible.append(case)
            for case in list(eligible):
                missing = set(case.depends) - {c.name for c in eligible}
                if missing:
                    report["skipped"][case.name] = f"Unavailable prerequisites: {sorted(missing)}"
                    eligible.remove(case)
            if not eligible:
                raise RuntimeError("No eligible workloads")
            rng = random.Random(args.seed)
            completed = set()
            deadline = time.monotonic() + args.duration
            pending = list(eligible)
            while time.monotonic() < deadline:
                if not pending:
                    pending = list(eligible)
                case = next_case(pending, completed, rng)
                print(f"RUN {case.name}", flush=True)
                run_random_pipeline(device, case, args, state, dai)
                completed.add(case.name)
            report["not_run"] = sorted({c.name for c in eligible} - completed)
            if report["not_run"]:
                print(f"Budget expired before full coverage: {report['not_run']}", flush=True)
                return 1
    except KeyboardInterrupt:
        report["interrupted"] = True
        return 130
    except Exception:
        report["error"] = traceback.format_exc()
        print(report["error"], flush=True)
        return 1
    finally:
        report["successful_nodes"] = sorted({node for run in report["runs"] if run["status"] == "passed" for node in run["nodes"]})
        successful_cases = {run["case"] for run in report["runs"] if run["status"] == "passed"}
        report["not_run"] = sorted({case.name for case in all_cases} - successful_cases - report["skipped"].keys())
        report["uncovered_example_nodes"] = sorted(set(inventory) - set(report["successful_nodes"]))
        (output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"Report: {output_dir / 'report.json'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
