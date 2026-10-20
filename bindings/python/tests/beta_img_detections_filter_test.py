import depthai as dai


def test_img_detections_filter_is_available_in_beta_node_namespace():
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.beta.node.ImgDetectionsFilter)

    assert isinstance(node, dai.beta.node.ImgDetectionsFilter)


def test_img_detections_filter_passes_detections_through_unchanged():
    with dai.Pipeline(False) as pipeline:
        node = pipeline.create(dai.beta.node.ImgDetectionsFilter)
        input_queue = node.input.createInputQueue()
        output_queue = node.output.createOutputQueue()

        detections = dai.ImgDetections()
        detections.setSequenceNum(42)
        detection = dai.ImgDetection()
        detection.label = 7
        detection.confidence = 0.75
        detections.detections = [detection]

        pipeline.start()
        input_queue.send(detections)
        output = output_queue.get(timeout=1.0)

    assert output is detections
    assert output.getSequenceNum() == 42
    assert len(output.detections) == 1
    assert output.detections[0].label == 7
    assert output.detections[0].confidence == 0.75
