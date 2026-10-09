"""Black-box cases P-1 to P-6 from img_detections_filter_test_spec.md."""

from datetime import timedelta

import depthai as dai
import numpy as np
import pytest


def transformation(width=512, height=512):
    intrinsics = [[width / 2, 0, width / 2], [0, width / 2, height / 2], [0, 0, 1]]
    extrinsics = dai.Extrinsics(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        dai.CameraBoardSocket.CAM_A,
    )
    return dai.ImgTransformation(width, height, intrinsics, dai.CameraModel.Perspective, [], extrinsics)


def message(t, detections):
    """Rows are (label, confidence, center x/y, width/height), all in pixels."""
    width, height = t.getSize()
    result = dai.ImgDetections()
    result.setTransformation(t)
    result.setSequenceNum(42)
    result.setTimestamp(timedelta(seconds=10))
    data = []
    for label, confidence, x, y, w, h in detections:
        detection = dai.ImgDetection()
        detection.label = label
        detection.confidence = confidence
        detection.setBoundingBox(
            dai.RotatedRect(dai.Point2f(x / width, y / height, True), dai.Size2f(w / width, h / height, True), 0)
        )
        data.append(detection)
    result.detections = data
    return result


def set_mask(msg, rows):
    mask = np.asarray(rows, dtype=np.uint8)
    if hasattr(msg, "setCvSegmentationMask"):
        msg.setCvSegmentationMask(mask)
    else:
        frame = dai.ImgFrame()
        frame.setType(dai.ImgFrame.Type.GRAY8)
        frame.setWidth(mask.shape[1])
        frame.setHeight(mask.shape[0])
        frame.setData(mask.ravel())
        msg.setSegmentationMask(frame)


def require_output(out, t, expected, mask=None, tolerance=1e-4):
    assert out is not None
    assert out.getTransformation() is not None
    assert out.getTransformation().isEqualTransformation(t)
    assert len(out.detections) == len(expected)
    width, height = t.getSize()
    for actual, (label, confidence, x, y, w, h) in zip(out.detections, expected):
        assert actual.label == label
        assert actual.confidence == pytest.approx(confidence, abs=1e-6, rel=0)
        box = actual.getBoundingBox().denormalize(width, height)
        assert (box.center.x, box.center.y, box.size.width, box.size.height, box.angle) == pytest.approx(
            (x, y, w, h, 0), abs=tolerance, rel=0
        )
    if mask is not None:
        expected_mask = np.asarray(mask, dtype=np.uint8)
        assert out.getSegmentationMaskWidth() == expected_mask.shape[1]
        assert out.getSegmentationMaskHeight() == expected_mask.shape[0]
        if hasattr(out, "getCvSegmentationMask"):
            actual_mask = out.getCvSegmentationMask()
        else:
            actual_mask = np.asarray(out.getMaskData(), dtype=np.uint8).reshape(expected_mask.shape)
        np.testing.assert_array_equal(actual_mask, expected_mask)


@pytest.mark.parametrize("_case", ["P-1[IN-1]"])
def test_p1_public_api(_case):
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        config = dai.ImgDetectionsFilterConfig()
        assert config is not None
        for name in ("inputs", "inputSourceMasks", "inputReference", "inputConfig", "out", "initialConfig"):
            assert hasattr(node, name)
        # Exercise the public node as well as inspecting its shape.
        queue = node.inputs["cam"].createInputQueue()
        output = node.out.createOutputQueue()
        pipeline.start()
        t = transformation()
        data = [(1, 0.9, 100, 100, 64, 64)]
        queue.send(message(t, data))
        require_output(output.get(timeout=timedelta(seconds=1)), t, data)


@pytest.mark.parametrize("_case", ["P-2[control]"])
def test_p2_beta_node_removed(_case):
    if hasattr(dai, "beta"):
        assert not hasattr(dai.beta.node, "ImgDetectionsFilter")


@pytest.mark.parametrize("_case", ["P-3[CF-1]"])
def test_p3_default_filter(_case):
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        queue = node.inputs["cam"].createInputQueue()
        output = node.out.createOutputQueue()
        pipeline.start()
        t = transformation()
        data = [(7, 0.75, 100, 100, 64, 64)]
        queue.send(message(t, data))
        out = output.get(timeout=timedelta(seconds=1))
        require_output(out, t, data)
        assert out.getSequenceNum() == 42


@pytest.mark.parametrize("_case", ["P-4[MK-1]"])
def test_p4_mask_reindexing(_case):
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        node.initialConfig.setConfidenceRange(0.5)
        queue = node.inputs["cam"].createInputQueue()
        output = node.out.createOutputQueue()
        t = transformation(8, 4)
        data = [(1, 0.9, 1, 2, 2, 4), (2, 0.25, 4, 2, 2, 4), (1, 0.75, 7, 2, 2, 4)]
        msg = message(t, data)
        set_mask(msg, [[0, 0, 255, 1, 1, 255, 2, 2]] * 3 + [[255, 255, 255, 1, 1, 255, 255, 255]])
        pipeline.start()
        queue.send(msg)
        require_output(
            output.get(timeout=timedelta(seconds=1)), t, [data[0], data[2]],
            [[0, 0, 255, 255, 255, 255, 1, 1]] * 3 + [[255] * 8],
        )


@pytest.mark.parametrize("_case", ["P-5[RT-1]"])
def test_p5_runtime_config_replacement(_case):
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        node.initialConfig.setConfidenceRange(0.5)
        queue = node.inputs["cam"].createInputQueue()
        output = node.out.createOutputQueue()
        t = transformation()
        data = [(1, 0.9, 100, 100, 64, 64), (2, 0.3, 300, 300, 64, 64)]
        pipeline.start()
        queue.send(message(t, data))
        require_output(output.get(timeout=timedelta(seconds=1)), t, [data[0]])
        config = dai.ImgDetectionsFilterConfig()
        config.labelsToReject = [1]
        node.inputConfig.send(config)
        queue.send(message(t, data))
        require_output(output.get(timeout=timedelta(seconds=1)), t, [data[1]])


@pytest.mark.parametrize("_case", ["P-6[OV-7][MK-15]"])
def test_p6_average_mask_union(_case):
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        t = transformation(8, 4)
        node.initialConfig.reference = t
        node.initialConfig.overlapMode = dai.ImgDetectionsFilterConfig.OverlapMode.AVERAGE
        in_a = node.inputs["a"].createInputQueue()
        in_b = node.inputs["b"].createInputQueue()
        output = node.out.createOutputQueue()
        a = message(t, [(0, 0.75, 3, 2, 4, 4)])
        b = message(t, [(0, 0.25, 4, 2, 4, 4)])
        set_mask(a, [
            [255, 255, 0, 0, 255, 255, 255, 255],
            [255, 0, 0, 0, 0, 255, 255, 255],
            [255, 0, 0, 0, 0, 255, 255, 255],
            [255, 0, 255, 255, 0, 255, 255, 255],
        ])
        set_mask(b, [
            [255, 255, 255, 0, 0, 255, 255, 255],
            [255, 255, 0, 0, 0, 0, 255, 255],
            [255, 255, 0, 0, 0, 0, 255, 255],
            [255, 255, 0, 255, 255, 0, 255, 255],
        ])
        pipeline.start()
        in_a.send(a)
        in_b.send(b)
        require_output(output.get(timeout=timedelta(seconds=1)), t, [(0, 0.75, 3.25, 2, 4, 4)], [
            [255, 255, 0, 0, 0, 255, 255, 255],
            [255, 0, 0, 0, 0, 0, 255, 255],
            [255, 0, 0, 0, 0, 0, 255, 255],
            [255, 0, 0, 255, 0, 0, 255, 255],
        ], tolerance=1e-3)


def test_source_mask_rejects_hidden_detection_before_nms():
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        t = transformation(8, 4)
        node.initialConfig.reference = t
        in_a = node.inputs["a"].createInputQueue()
        in_b = node.inputs["b"].createInputQueue()
        masks = node.inputSourceMasks["a"].createInputQueue()
        output = node.out.createOutputQueue()
        mask = dai.ImgFrame()
        mask.setType(dai.ImgFrame.Type.GRAY8)
        mask.setWidth(8)
        mask.setHeight(4)
        mask.setData(np.zeros(32, dtype=np.uint8))
        mask.setTransformation(t)
        pipeline.start()
        masks.send(mask)
        for _ in range(2):
            in_a.send(message(t, [(1, 0.9, 3, 2, 4, 4)]))
            in_b.send(message(t, [(1, 0.8, 4, 2, 4, 4)]))
            require_output(output.get(timeout=timedelta(seconds=1)), t, [(1, 0.8, 4, 2, 4, 4)])
