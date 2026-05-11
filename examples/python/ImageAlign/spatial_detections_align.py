#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


CAM_A = dai.CameraBoardSocket.CAM_A
CAM_B = dai.CameraBoardSocket.CAM_B
CAM_C = dai.CameraBoardSocket.CAM_C
STEREO_SIZE = (640, 400)


def get_model_setup(device: dai.Device):
    fps = 20.0
    model_name = "luxonis/yolov8-large-pose-estimation:coco-640x352"
    if device.getPlatform() == dai.Platform.RVC2:
        fps = 10.0
        model_name = "luxonis/yolov8-nano-pose-estimation:coco-512x288"

    model_description = dai.NNModelDescription(model_name)
    model_path = dai.getModelFromZoo(dai.NNModelDescription(model_name, platform=device.getPlatformAsString()))
    model_archive = dai.NNArchive(model_path)
    input_size = model_archive.getInputSize()

    dai_type = model_archive.getConfig().model.inputs[0].preprocessing.daiType
    frame_type = None
    if dai_type:
        try:
            frame_type = getattr(dai.ImgFrame.Type, dai_type)
        except AttributeError:
            frame_type = None

    if frame_type is None:
        if device.getPlatform() == dai.Platform.RVC2:
            frame_type = dai.ImgFrame.Type.BGR888p
        else:
            frame_type = dai.ImgFrame.Type.BGR888i

    return fps, model_name, model_description, model_archive, input_size, frame_type


def frame_norm(frame: np.ndarray, coords) -> np.ndarray:
    norm_vals = np.full(len(coords), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(coords), 0, 1) * norm_vals).astype(int)


def colorize_depth(depth_frame: np.ndarray) -> np.ndarray:
    invalid_mask = depth_frame == 0
    try:
        min_depth = np.percentile(depth_frame[depth_frame != 0], 3)
        max_depth = np.percentile(depth_frame[depth_frame != 0], 95)
        log_depth = np.zeros_like(depth_frame, dtype=np.float32)
        np.log(depth_frame, where=depth_frame != 0, out=log_depth)
        log_min_depth = np.log(min_depth)
        log_max_depth = np.log(max_depth)
        np.nan_to_num(log_depth, copy=False, nan=log_min_depth)
        log_depth = np.clip(log_depth, log_min_depth, log_max_depth)

        depth_frame_color = np.interp(log_depth, (log_min_depth, log_max_depth), (0, 255))
        depth_frame_color = np.nan_to_num(depth_frame_color).astype(np.uint8)
        depth_frame_color = cv2.applyColorMap(depth_frame_color, cv2.COLORMAP_JET)
        depth_frame_color[invalid_mask] = 0
        return depth_frame_color
    except IndexError:
        return np.zeros((depth_frame.shape[0], depth_frame.shape[1], 3), dtype=np.uint8)


def get_label(detection, label_map: list[str]) -> str:
    if detection.labelName:
        return detection.labelName
    if detection.label < len(label_map):
        return label_map[detection.label]
    return str(detection.label)


def draw_detection_set(frame: np.ndarray, detections, label_map: list[str], color, draw_spatial: bool) -> None:
    for detection in detections.detections:
        bbox = detection.getBoundingBox().denormalize(frame.shape[1], frame.shape[0])
        points = np.array([[int(point.x), int(point.y)] for point in bbox.getPoints()], dtype=np.int32)
        anchor = tuple(points[0])

        cv2.polylines(frame, [points], isClosed=True, color=color, thickness=2)
        cv2.putText(frame, get_label(detection, label_map), (anchor[0] + 8, anchor[1] + 18), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(
            frame,
            f"{int(detection.confidence * 100)}%",
            (anchor[0] + 8, anchor[1] + 38),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            color,
        )

        if draw_spatial:
            coords = detection.spatialCoordinates
            cv2.putText(
                frame,
                f"Z: {int(coords.z / 10)} cm",
                (anchor[0] + 8, anchor[1] + 58),
                cv2.FONT_HERSHEY_TRIPLEX,
                0.5,
                color,
            )

        keypoints = detection.getKeypoints()
        for keypoint in keypoints:
            keypoint_pos = frame_norm(frame, (keypoint.imageCoordinates.x, keypoint.imageCoordinates.y))
            cv2.circle(frame, tuple(keypoint_pos), 3, color, -1)

        for edge in detection.getEdges():
            if edge[0] >= len(keypoints) or edge[1] >= len(keypoints):
                continue
            kp1 = keypoints[edge[0]]
            kp2 = keypoints[edge[1]]
            kp1_pos = frame_norm(frame, (kp1.imageCoordinates.x, kp1.imageCoordinates.y))
            kp2_pos = frame_norm(frame, (kp2.imageCoordinates.x, kp2.imageCoordinates.y))
            cv2.line(frame, tuple(kp1_pos), tuple(kp2_pos), color, 2)


device = dai.Device()
fps, model_name, model_description, model_archive, input_size, input_frame_type = get_model_setup(device)

required_cam_capabilities = dai.ImgFrameCapability()
required_cam_capabilities.fps.fixed(fps)
required_cam_capabilities.enableUndistortion = True


with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    camera_a = pipeline.create(dai.node.Camera).build(CAM_A, sensorFps=fps)
    detections_a = pipeline.create(dai.node.DetectionNetwork).build(camera_a, model_description, required_cam_capabilities)

    camera_b = pipeline.create(dai.node.Camera).build(CAM_B, sensorFps=fps)
    camera_c = pipeline.create(dai.node.Camera).build(CAM_C, sensorFps=fps)

    camera_b_stereo = camera_b.requestOutput((1280, 800), fps=fps)
    camera_b_nn = camera_b.requestOutput((800, 640), fps=fps)
    camera_c_stereo = camera_c.requestOutput((1280, 800), fps=fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    camera_b_stereo.link(stereo.left)
    camera_c_stereo.link(stereo.right)

    spatial_calculator = pipeline.create(dai.node.SpatialLocationCalculator)
    spatial_calculator.initialConfig.setCalculateSpatialKeypoints(True)
    detections_a.out.link(spatial_calculator.inputDetections)

    if device.getPlatform() == dai.Platform.RVC4:
        depth_align_a = pipeline.create(dai.node.ImageAlign)
        stereo.depth.link(depth_align_a.input)
        detections_a.passthrough.link(depth_align_a.inputAlignTo)
        depth_align_a.outputAligned.link(spatial_calculator.inputDepth)
    else:
        detections_a.passthrough.link(stereo.inputAlignTo)
        stereo.depth.link(spatial_calculator.inputDepth)

    camera_b_preproc = pipeline.create(dai.node.ImageManip)
    camera_b_preproc.initialConfig.setOutputSize(input_size[0], input_size[1])
    camera_b_preproc.initialConfig.setFrameType(input_frame_type)
    camera_b_preproc.setMaxOutputFrameSize(input_size[0] * input_size[1] * 3 + 1024)
    camera_b_nn.link(camera_b_preproc.inputImage)

    detections_b = pipeline.create(dai.node.DetectionNetwork).build(camera_b, model_description)
    label_map = detections_b.getClasses()

    generic_align = pipeline.create(dai.node.ImageAlign)
    spatial_calculator.outputDetections.link(generic_align.input)
    detections_b.out.link(generic_align.inputAlignTo)
    generic_align.setRunOnHost(True)

    original_frame_queue = detections_a.passthrough.createOutputQueue()
    original_spatial_queue = spatial_calculator.outputDetections.createOutputQueue()
    frame_queue = detections_b.passthrough.createOutputQueue()
    detections_b_queue = detections_b.out.createOutputQueue()
    aligned_spatial_queue = generic_align.outputAligned.createOutputQueue()

    pipeline.start()
    print("Pipeline created.")

    while pipeline.isRunning():
        original_frame_a = original_frame_queue.get()
        original_spatial = original_spatial_queue.get()
        frame_b = frame_queue.get()
        detections_on_b = detections_b_queue.get()
        aligned_spatial = aligned_spatial_queue.get()

        assert isinstance(original_frame_a, dai.ImgFrame)
        assert isinstance(original_spatial, dai.SpatialImgDetections)
        assert isinstance(frame_b, dai.ImgFrame)
        assert isinstance(detections_on_b, dai.ImgDetections)
        assert isinstance(aligned_spatial, dai.SpatialImgDetections)

        original_frame = original_frame_a.getCvFrame()
        if len(original_frame.shape) == 2:
            original_frame = cv2.cvtColor(original_frame, cv2.COLOR_GRAY2BGR)
        draw_detection_set(original_frame, original_spatial, label_map, (0, 0, 255), draw_spatial=True)

        frame = frame_b.getCvFrame()
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        draw_detection_set(frame, detections_on_b, label_map, (0, 255, 0), draw_spatial=False)
        draw_detection_set(frame, aligned_spatial, label_map, (0, 0, 255), draw_spatial=True)

        cv2.imshow("CAM_A original spatial detections", original_frame)
        cv2.imshow("CAM_B aligned detections", frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
