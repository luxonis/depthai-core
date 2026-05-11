#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np


MODEL_DESCRIPTION = dai.NNModelDescription("yolov6-nano")
FPS_RVC2 = 10.0
FPS_DEFAULT = 20.0
CROP_OUTPUT_SIZE = 320


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


def get_model_archive(device: dai.Device) -> tuple[dai.NNArchive, tuple[int, int], dai.ImgFrame.Type]:
    model_path = dai.getModelFromZoo(dai.NNModelDescription("yolov6-nano", platform=device.getPlatformAsString()))
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

    return model_archive, input_size, frame_type


class DetectionCropAlignRouter(dai.node.ThreadedHostNode):
    def __init__(self, crop_output_size: int):
        super().__init__()
        self.crop_output_size = crop_output_size

        self.inputDetections = self.createInput()
        self.inputDepth = self.createInput()
        self.outputCropConfig = self.createOutput()
        self.outputAlignTo = self.createOutput()
        self.outputDepth = self.createOutput()
        self.outputCount = self.createOutput()

        self.inputDetections.setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, True)])
        self.inputDepth.setPossibleDatatypes([(dai.DatatypeEnum.ImgFrame, True)])
        self.outputCropConfig.setPossibleDatatypes([(dai.DatatypeEnum.ImageManipConfig, True)])
        self.outputAlignTo.setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, True)])
        self.outputDepth.setPossibleDatatypes([(dai.DatatypeEnum.ImgFrame, True)])
        self.outputCount.setPossibleDatatypes([(dai.DatatypeEnum.Buffer, True)])

    def _detection_to_source_rect(self, detections: dai.ImgDetections, detection: dai.ImgDetection) -> dai.RotatedRect:
        detection_transform = detections.getTransformation()
        assert detection_transform is not None

        norm_width, norm_height = detection_transform.getSize()
        detection_rect = detection.getBoundingBox().denormalize(norm_width, norm_height)
        return detection_transform.invTransformRect(detection_rect)

    def _build_crop_transform(self, source_transform: dai.ImgTransformation, source_rect: dai.RotatedRect) -> dai.ImgTransformation:
        source_width, source_height = source_transform.getSourceSize()
        crop_transform = dai.ImgTransformation(source_width, source_height)
        crop_transform.setIntrinsicMatrix(source_transform.getSourceIntrinsicMatrix())
        crop_transform.setDistortionModel(source_transform.getDistortionModel())
        crop_transform.setDistortionCoefficients(source_transform.getDistortionCoefficients())
        crop_transform.setExtrinsics(source_transform.getExtrinsics())
        crop_transform.addSrcCrops([source_rect])

        # Mirror ImageManip's rotated crop + letterbox target geometry so ImageAlign uses the same transform.
        src_points = np.array([[point.x, point.y] for point in source_rect.getPoints()], dtype=np.float32)
        scale = min(self.crop_output_size / source_rect.size.width, self.crop_output_size / source_rect.size.height)
        dst_width = source_rect.size.width * scale
        dst_height = source_rect.size.height * scale
        pad_x = (self.crop_output_size - dst_width) / 2.0
        pad_y = (self.crop_output_size - dst_height) / 2.0

        dst_points = np.array(
            [
                [pad_x, pad_y],
                [pad_x + dst_width, pad_y],
                [pad_x + dst_width, pad_y + dst_height],
                [pad_x, pad_y + dst_height],
            ],
            dtype=np.float32,
        )
        matrix = cv2.getPerspectiveTransform(src_points, dst_points).astype(np.float32)
        crop_transform.addTransformation(matrix.tolist())
        crop_transform.setSize(self.crop_output_size, self.crop_output_size)
        return crop_transform

    def _create_crop_config(self, source_rect: dai.RotatedRect, reuse_previous_image: bool) -> dai.ImageManipConfig:
        config = dai.ImageManipConfig()
        config.addCropRotatedRect(source_rect, False)
        config.setOutputSize(self.crop_output_size, self.crop_output_size, dai.ImageManipConfig.ResizeMode.LETTERBOX)
        config.setReusePreviousImage(reuse_previous_image)
        return config

    def _create_align_target(self, crop_transform: dai.ImgTransformation, detections: dai.ImgDetections) -> dai.ImgDetections:
        align_target = dai.ImgDetections()
        align_target.setTransformation(crop_transform)
        align_target.setTimestamp(detections.getTimestamp())
        align_target.setTimestampDevice(detections.getTimestampDevice())
        align_target.setSequenceNum(detections.getSequenceNum())
        return align_target

    def _create_count_buffer(self, detections: dai.ImgDetections, count: int) -> dai.Buffer:
        count_buffer = dai.Buffer()
        count_buffer.setTimestamp(detections.getTimestamp())
        count_buffer.setTimestampDevice(detections.getTimestampDevice())
        count_buffer.setSequenceNum(detections.getSequenceNum())
        count_buffer.setData(np.array([count], dtype=np.int32).tobytes())
        return count_buffer

    def run(self):
        while self.mainLoop():
            depth_frame = self.inputDepth.get()
            detections = self.inputDetections.get()

            if detections.getTransformation() is None:
                self.outputCount.send(self._create_count_buffer(detections, 0))
                continue

            sent_count = 0
            reuse_previous_image = False
            for detection in detections.detections:
                source_rect = self._detection_to_source_rect(detections, detection)
                if source_rect.size.width <= 1.0 or source_rect.size.height <= 1.0:
                    continue

                crop_transform = self._build_crop_transform(detections.getTransformation(), source_rect)
                # Emit one crop config, one duplicated depth frame, and one crop-space align target per detection.
                self.outputCropConfig.send(self._create_crop_config(source_rect, reuse_previous_image))
                self.outputDepth.send(depth_frame)
                self.outputAlignTo.send(self._create_align_target(crop_transform, detections))

                reuse_previous_image = True
                sent_count += 1

            self.outputCount.send(self._create_count_buffer(detections, sent_count))


device = dai.Device()
fps = FPS_RVC2 if device.getPlatform() == dai.Platform.RVC2 else FPS_DEFAULT
model_archive, input_size, input_frame_type = get_model_archive(device)

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=fps)
    full_res_stream = camera.requestOutput((1920, 1080), enableUndistortion=True)

    nn_resize = pipeline.create(dai.node.ImageManip)
    nn_resize.initialConfig.setOutputSize(input_size[0], input_size[1])
    nn_resize.initialConfig.setFrameType(input_frame_type)
    full_res_stream.link(nn_resize.inputImage)

    detections_nn = pipeline.create(dai.node.DetectionNetwork).build(nn_resize.out, model_archive)

    mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    mono_left.requestFullResolutionOutput().link(stereo.left)
    mono_right.requestFullResolutionOutput().link(stereo.right)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    crop_manip = pipeline.create(dai.node.ImageManip)
    crop_manip.setMaxOutputFrameSize(CROP_OUTPUT_SIZE * CROP_OUTPUT_SIZE * 3 + 1024)
    crop_manip.initialConfig.setOutputSize(CROP_OUTPUT_SIZE, CROP_OUTPUT_SIZE, dai.ImageManipConfig.ResizeMode.LETTERBOX)
    crop_manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    crop_manip.inputImage.setBlocking(False)
    crop_manip.inputImage.setMaxSize(1)
    full_res_stream.link(crop_manip.inputImage)

    router = pipeline.create(DetectionCropAlignRouter, CROP_OUTPUT_SIZE)
    detections_nn.out.link(router.inputDetections)
    stereo.depth.link(router.inputDepth)
    router.outputCropConfig.link(crop_manip.inputConfig)

    align = pipeline.create(dai.node.ImageAlign)
    router.outputDepth.link(align.input)
    router.outputAlignTo.link(align.inputAlignTo)

    crop_queue = crop_manip.out.createOutputQueue()
    aligned_depth_queue = align.outputAligned.createOutputQueue()
    count_queue = router.outputCount.createOutputQueue()

    pipeline.start()
    print("Pipeline created.")

    previous_count = 0
    while pipeline.isRunning():
        count_buffer = count_queue.get()
        count = int(np.frombuffer(count_buffer.getData(), dtype=np.int32)[0])

        crop_frames = []
        aligned_depth_frames = []
        for _ in range(count):
            crop_frame = crop_queue.get()
            aligned_depth = aligned_depth_queue.get()

            assert isinstance(crop_frame, dai.ImgFrame)
            assert isinstance(aligned_depth, dai.ImgFrame)

            crop_frames.append(crop_frame.getCvFrame())
            aligned_depth_frames.append(colorize_depth(aligned_depth.getFrame()))

        for index, crop_frame in enumerate(crop_frames):
            cv2.imshow(f"Crop {index}", crop_frame)

        for index, aligned_depth_frame in enumerate(aligned_depth_frames):
            cv2.imshow(f"Aligned depth crop {index}", aligned_depth_frame)

        for index in range(count, previous_count):
            cv2.destroyWindow(f"Crop {index}")
            cv2.destroyWindow(f"Aligned depth crop {index}")

        previous_count = count

        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
