import depthai as dai


def make_lines_message():
    first = dai.beta.Line()
    first.startPoint = dai.Point2f(0.10, 0.20, True)
    first.endPoint = dai.Point2f(0.30, 0.40, True)
    first.confidence = 0.90

    second = dai.beta.Line()
    second.startPoint = dai.Point2f(0.50, 0.60, True)
    second.endPoint = dai.Point2f(0.70, 0.80, True)
    second.confidence = 0.75

    message = dai.beta.Lines()
    message.setSequenceNum(42)
    transformation = dai.ImgTransformation(640, 400)
    transformation.addCrop(12, 8, 320, 200).addScale(0.75, 0.50)
    message.setTransformation(transformation)
    message.lines = [first, second]
    return message


def assert_roundtrip(expected, actual):
    if not isinstance(actual, dai.beta.Lines):
        raise RuntimeError(f"Expected dai.beta.Lines, received {type(actual).__name__}")
    if actual.getSequenceNum() != expected.getSequenceNum():
        raise RuntimeError("Sequence number changed during round trip")

    expected_transformation = expected.getTransformation()
    actual_transformation = actual.getTransformation()
    if (actual_transformation is None) != (expected_transformation is None):
        raise RuntimeError("ImgTransformation presence changed during round trip")
    if expected_transformation is not None and not actual_transformation.isEqualTransformation(expected_transformation):
        raise RuntimeError("ImgTransformation changed during round trip")

    if len(actual.lines) != len(expected.lines):
        raise RuntimeError("Line count changed during round trip")

    for index, (expected_line, actual_line) in enumerate(zip(expected.lines, actual.lines)):
        expected_values = (
            expected_line.startPoint.x,
            expected_line.startPoint.y,
            expected_line.endPoint.x,
            expected_line.endPoint.y,
            expected_line.confidence,
        )
        actual_values = (
            actual_line.startPoint.x,
            actual_line.startPoint.y,
            actual_line.endPoint.x,
            actual_line.endPoint.y,
            actual_line.confidence,
        )
        if actual_values != expected_values:
            raise RuntimeError(f"Line {index} changed during round trip: {actual_values} != {expected_values}")


with dai.Pipeline() as pipeline:
    script = pipeline.create(dai.node.Script)
    input_queue = script.inputs["in"].createInputQueue()
    output_queue = script.outputs["out"].createOutputQueue()

    # The Script node does not inspect or modify the Lines message.
    script.setScript(
        """
        while True:
            node.outputs["out"].send(node.inputs["in"].get())
        """
    )

    pipeline.start()
    sent = make_lines_message()
    input_queue.send(sent)
    received = output_queue.get()
    if received is None:
        raise RuntimeError("Timed out waiting for Lines round trip")

    assert_roundtrip(sent, received)
    print(f"Lines round trip succeeded: {len(received.lines)} lines, sequence {received.getSequenceNum()}")
