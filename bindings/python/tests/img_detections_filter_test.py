from datetime import timedelta

import depthai as dai
import pytest


def test_public_filter_and_runtime_mask_reindexing():
    if hasattr(dai, "beta"):
        assert not hasattr(dai.beta.node, "ImgDetectionsFilter")
        assert not hasattr(dai.beta, "ImgDetectionsFilterConfig")
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        node.inputs["unused"]
        queue = node.inputs["cam"].createInputQueue()
        configs = node.inputConfig.createInputQueue()
        output = node.out.createOutputQueue()
        pipeline.start()
        message = dai.ImgDetections()
        a, b = dai.ImgDetection(), dai.ImgDetection()
        a.label, a.confidence = 1, 0.9
        b.label, b.confidence = 2, 0.7
        message.detections = [a, b]
        mask = dai.ImgFrame()
        mask.setType(dai.ImgFrame.Type.GRAY8)
        mask.setWidth(4)
        mask.setHeight(1)
        mask.setData([0, 1, 255, 9])
        message.setSegmentationMask(mask)
        config = dai.ImgDetectionsFilterConfig()
        config.labelsToKeep = [1, 2]
        config.labelsToReject = [1]
        node.inputConfig.send(config)
        queue.send(message)
        result = output.get(timedelta(seconds=2))
        assert result is not None
        assert [d.label for d in result.detections] == [2]
        assert list(result.getMaskData()) == [255, 0, 255, 255]
        assert list(message.getMaskData()) == [0, 1, 255, 9]


def test_config_ranges_and_nested_alias():
    config = dai.node.ImgDetectionsFilter.Config()
    config.setConfidenceRange(0.2, 0.9).setSizeRange(10, 500).setWidthRange(1, 50).setHeightRange(2, 100)
    assert config.validate()
    assert config.hasGeometryFilters()
    config.overlapMode = dai.ImgDetectionsFilterConfig.OverlapMode.AVERAGE
    assert config.overlapMode == dai.node.ImgDetectionsFilter.Config.OverlapMode.AVERAGE
    config.setConfidenceRange(0.5, 0.5)
    assert not config.validate()


@pytest.mark.parametrize("spread_ms,threshold_ms", [(0, 40), (65, 100)])
def test_synchronized_demux_to_filter_example_topology(spread_ms, threshold_ms):
    with dai.Pipeline(False) as pipeline:
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(True)
        sync.setSyncThreshold(timedelta(milliseconds=threshold_ms))
        demux = pipeline.create(dai.node.MessageDemux)
        demux.setRunOnHost(True)
        sync.out.link(demux.input)
        node = pipeline.create(dai.node.ImgDetectionsFilter)
        node.initialConfig.reference = dai.ImgTransformation(200, 100)
        queues = []
        for key in ("left", "right"):
            queues.append(sync.inputs[key].createInputQueue())
            demux.outputs[key].setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, False)])
            demux.outputs[key].link(node.inputs[key])
        output = node.out.createOutputQueue()
        pipeline.start()
        for index, queue in enumerate(queues):
            message = dai.ImgDetections()
            message.setTransformation(dai.ImgTransformation(200, 100))
            message.setTimestamp(timedelta(seconds=1, milliseconds=index * spread_ms))
            detection = dai.ImgDetection()
            detection.setOuterBoundingBox(0.1, 0.1, 0.8, 0.8)
            detection.confidence = 0.9 - index * 0.1
            message.detections = [detection]
            queue.send(message)
        result = output.get(timedelta(seconds=2))
        assert result is not None
        assert len(result.detections) == 1
        assert abs(result.detections[0].confidence - 0.9) < 1e-6
        assert result.getTimestamp() == timedelta(seconds=1, milliseconds=spread_ms)
