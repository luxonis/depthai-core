#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


MODEL_DESCRIPTION = dai.NNModelDescription("yolov6-nano")
FPS_RVC2 = 10.0
FPS_DEFAULT = 20.0


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


def frame_norm(frame: np.ndarray, bbox: tuple[float, float, float, float]) -> np.ndarray:
    norm_vals = np.full(len(bbox), frame.shape[0])
    norm_vals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)


def draw_detections(frame: np.ndarray, detections: dai.ImgDetections, label_map: list[str]) -> None:
    color = (0, 255, 0)
    for detection in detections.detections:
        bbox = frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
        label = label_map[detection.label] if detection.label < len(label_map) else str(detection.label)

        cv2.putText(frame, label, (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)


device = dai.Device()
fps = FPS_RVC2 if device.getPlatform() == dai.Platform.RVC2 else FPS_DEFAULT

required_cam_capabilities = dai.ImgFrameCapability()
required_cam_capabilities.fps.fixed(fps)
required_cam_capabilities.enableUndistortion = True

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    detections_nn = pipeline.create(dai.node.DetectionNetwork).build(camera, MODEL_DESCRIPTION, required_cam_capabilities)
    label_map = detections_nn.getClasses()

    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    mono_left.requestFullResolutionOutput().link(stereo.left)
    mono_right.requestFullResolutionOutput().link(stereo.right)

    align = pipeline.create(dai.node.ImageAlign)
    align.setRunOnHost(True)
    stereo.depth.link(align.input)
    detections_nn.out.link(align.inputAlignTo)

    depth_queue = align.outputAligned.createOutputQueue()
    detections_queue = detections_nn.out.createOutputQueue()

    pipeline.start()
    print("Pipeline created.")

    while pipeline.isRunning():
        aligned_depth = depth_queue.get()
        detections = detections_queue.get()

        print(type(aligned_depth), type(detections))
        assert isinstance(aligned_depth, dai.ImgFrame)
        assert isinstance(detections, dai.ImgDetections)

        depth_frame = colorize_depth(aligned_depth.getFrame())
        draw_detections(depth_frame, detections, label_map)

        cv2.imshow("Aligned depth detections", depth_frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
