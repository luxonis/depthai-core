#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


ROTATED_SIZE = (960, 720)
ROTATION_DEG = 30.0


def get_model_setup(device: dai.Device):
    fps = 20.0
    model_name = "luxonis/yolov8-instance-segmentation-large:coco-640x480"
    if device.getPlatform() == dai.Platform.RVC2:
        fps = 10.0
        model_name = "luxonis/yolov8-instance-segmentation-nano:coco-512x288"

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

    return fps, model_name, model_archive, input_size, frame_type


def overlay_segmentation(image: np.ndarray, detections: dai.ImgDetections) -> np.ndarray:
    output = image.copy()
    segmentation_mask = detections.getCvSegmentationMask()
    if segmentation_mask is not None:
        mask = segmentation_mask.copy()
        scaled_mask = np.zeros_like(mask, dtype=np.uint8)
        valid_mask = mask != 255
        scaled_mask[valid_mask] = ((mask[valid_mask].astype(np.uint16) * 37) % 256).astype(np.uint8)
        colored_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
        colored_mask[~valid_mask] = output[~valid_mask]
        output = cv2.addWeighted(output, 0.7, colored_mask, 0.3, 0)
    return output


def clip_polygon_to_image(points: np.ndarray, width: int, height: int) -> np.ndarray:
    def inside(point: np.ndarray, edge: str) -> bool:
        x, y = point
        if edge == "left":
            return x >= 0
        if edge == "right":
            return x <= width - 1
        if edge == "top":
            return y >= 0
        return y <= height - 1

    def intersect(start: np.ndarray, end: np.ndarray, edge: str) -> np.ndarray:
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        if edge == "left":
            x = 0.0
            t = 0.0 if dx == 0 else (x - x1) / dx
            return np.array([x, y1 + t * dy], dtype=np.float32)
        if edge == "right":
            x = float(width - 1)
            t = 0.0 if dx == 0 else (x - x1) / dx
            return np.array([x, y1 + t * dy], dtype=np.float32)
        if edge == "top":
            y = 0.0
            t = 0.0 if dy == 0 else (y - y1) / dy
            return np.array([x1 + t * dx, y], dtype=np.float32)

        y = float(height - 1)
        t = 0.0 if dy == 0 else (y - y1) / dy
        return np.array([x1 + t * dx, y], dtype=np.float32)

    clipped = [point.astype(np.float32) for point in points]
    for edge in ("left", "right", "top", "bottom"):
        if not clipped:
            break

        output = []
        prev = clipped[-1]
        prev_inside = inside(prev, edge)
        for curr in clipped:
            curr_inside = inside(curr, edge)
            if curr_inside:
                if not prev_inside:
                    output.append(intersect(prev, curr, edge))
                output.append(curr)
            elif prev_inside:
                output.append(intersect(prev, curr, edge))
            prev = curr
            prev_inside = curr_inside
        clipped = output

    if not clipped:
        return np.empty((0, 2), dtype=np.int32)

    return np.rint(np.array(clipped, dtype=np.float32)).astype(np.int32)


def draw_detections(image: np.ndarray, detections: dai.ImgDetections, label_map: list[str]) -> None:
    for detection in detections.detections:
        bbox = detection.getBoundingBox().denormalize(image.shape[1], image.shape[0])
        points = np.array([[point.x, point.y] for point in bbox.getPoints()], dtype=np.float32)
        clipped_points = clip_polygon_to_image(points, image.shape[1], image.shape[0])
        if clipped_points.size == 0:
            continue

        anchor = tuple(clipped_points[0])
        label = detection.labelName if detection.labelName else (label_map[detection.label] if detection.label < len(label_map) else str(detection.label))

        if len(clipped_points) >= 3:
            cv2.polylines(image, [clipped_points], isClosed=True, color=(255, 255, 255), thickness=2)
        elif len(clipped_points) == 2:
            cv2.line(image, tuple(clipped_points[0]), tuple(clipped_points[1]), (255, 255, 255), 2)
        cv2.putText(image, label, (anchor[0] + 8, anchor[1] + 18), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255))
        cv2.putText(
            image,
            f"{int(detection.confidence * 100)}%",
            (anchor[0] + 8, anchor[1] + 38),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            (255, 255, 255),
        )
model_name = "luxonis/yolov8-instance-segmentation-large:coco-640x480"

device = dai.Device()
fps, model_name, model_archive, input_size, input_frame_type = get_model_setup(device)

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=fps)
    # full_res_stream = camera.requestOutput((1000, 500), enableUndistortion=True)
    full_res_stream = camera.requestOutput((1000, 500), resizeMode=dai.ImgResizeMode.CROP, enableUndistortion=True)

    # nn_resize = pipeline.create(dai.node.ImageManip)
    # nn_resize.initialConfig.setOutputSize(input_size[0], input_size[1])
    # nn_resize.initialConfig.setFrameType(input_frame_type)
    # nn_resize.setMaxOutputFrameSize(input_size[0] * input_size[1] * 3 + 1024)
    # full_res_stream.link(nn_resize.inputImage)

    rotated_rgb = pipeline.create(dai.node.ImageManip)
    rotated_rgb.initialConfig.addRotateDeg(30.0)
    rotated_rgb.initialConfig.setOutputSize(800, 600)
    rotated_rgb.initialConfig.setBackgroundColor(255, 255, 255)
    # rotated_rgb.initialConfig.setOutputSize(960, 720, mode=dai.ImageManipConfig.ResizeMode.CENTER_CROP)
    
    # rotated_rgb.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    rotated_rgb.setMaxOutputFrameSize(960 * 720 * 3 + 1024)
    full_res_stream.link(rotated_rgb.inputImage)

    det_nn = pipeline.create(dai.node.DetectionNetwork).build(camera, model_name)
    label_map = det_nn.getClasses()

    align = pipeline.create(dai.node.ImageAlign)
    align.setRunOnHost(True)
    det_nn.out.link(align.input)
    rotated_rgb.out.link(align.inputAlignTo)

    rotated_rgb_queue = rotated_rgb.out.createOutputQueue()
    aligned_detections_queue = align.outputAligned.createOutputQueue()
    det_nn_passthrough_queue = det_nn.passthrough.createOutputQueue()
    det_nn_dets_queue = det_nn.out.createOutputQueue()

    pipeline.start()
    print("Pipeline created.")

    while pipeline.isRunning():
        rotated_frame = rotated_rgb_queue.get()
        aligned_detections = aligned_detections_queue.get()
        det_nn_passthrough_frame = det_nn_passthrough_queue.get()
        det_nn_dets = det_nn_dets_queue.get()
        
        assert isinstance(det_nn_passthrough_frame, dai.ImgFrame)

        assert isinstance(rotated_frame, dai.ImgFrame)
        assert isinstance(aligned_detections, dai.ImgDetections)
        
        rgb_passthrough_frame = det_nn_passthrough_frame.getCvFrame()
        draw_detections(rgb_passthrough_frame, det_nn_dets, label_map)

        frame = rotated_frame.getCvFrame()
        frame = overlay_segmentation(frame, aligned_detections)
        draw_detections(frame, aligned_detections, label_map)

        cv2.imshow("Rotated RGB segmentation align", frame)
        cv2.imshow("RGB passthrough with detections", rgb_passthrough_frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
