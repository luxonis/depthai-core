import depthai as dai

message_types = [
    dai.ADatatype,
    dai.AprilTagConfig,
    dai.AprilTags,
    dai.BenchmarkReport,
    dai.Buffer,
    dai.CameraControl,
    dai.EdgeDetectorConfig,
    dai.FeatureTrackerConfig,
    dai.ImageManipConfig,
    dai.ImgDetections,
    dai.ImgFrame,
    dai.IMUData,
    dai.MessageGroup,
    dai.NNData,
    dai.PointCloudConfig,
    dai.PointCloudData,
    dai.SpatialImgDetections,
    dai.SpatialLocationCalculatorConfig,
    dai.SpatialLocationCalculatorData,
    dai.StereoDepthConfig,
    dai.SystemInformation,
    dai.SystemInformationRVC4,
    dai.ToFConfig,
    dai.TrackedFeatures,
    dai.Tracklets
]

transformable_message_types = [
    dai.AprilTags,
    dai.ImgDetections,
    dai.PointCloudData,
    dai.SegmentationMask,
    dai.SpatialImgDetections,
    dai.Tracklets,
]

def test_constructable():
    for message_type in message_types:
        message = message_type()
        assert message is not None

def test_transformable_messages_are_instances_of_both_bases():
    for message_type in transformable_message_types:
        message = message_type()
        assert isinstance(message, dai.Buffer)
        assert isinstance(message, dai.Transformable)


def test_messages_can_be_inherited():
    for message_type in message_types:
        class CustomMessage(message_type):
            def __init__(self):
                super().__init__()
                self.test_field = 42

        message = CustomMessage()
        assert message is not None
        assert isinstance(message, message_type)
        assert message.test_field == 42

def test_python_is_kept_alive():
    import gc
    for message_type in message_types:
        class CustomMessage(message_type):
            def __init__(self, number=0):
                super().__init__()
                self.test_field = number

        messageQueue = dai.MessageQueue(maxSize=10, blocking=True)
        for i in range(10):
            messageQueue.send(CustomMessage(i))
        gc.collect() # Force garbage collection
        for i in range(10):
            message : CustomMessage = messageQueue.get()
            assert isinstance(message, CustomMessage)
            assert message.test_field == i

def test_transformable_buffer_dispatches_to_python_override():
    class CustomTransformableBuffer(dai.TransformableBuffer):
        def __init__(self):
            super().__init__()
            self.override_called = False

        def transformTo(self, target):
            self.override_called = True
            transformed = CustomTransformableBuffer()
            transformed.setTransformation(target)
            transformed.override_called = True
            return transformed

    source = dai.ImgTransformation(640, 480)
    target = dai.ImgTransformation(1280, 720)

    message = CustomTransformableBuffer()
    message.setTransformation(source)

    transformed = dai.TransformableBuffer.transformTo(message, target)

    assert isinstance(message, dai.Buffer)
    assert isinstance(message, dai.Transformable)
    assert isinstance(message, dai.TransformableBuffer)
    assert isinstance(transformed, CustomTransformableBuffer)
    assert message.override_called
    assert message.getTransformation().isEqualTransformation(source)
    assert transformed.getTransformation().isEqualTransformation(target)

def test_with_host_nodes():
    import time
    class MyCustomMessage(dai.Buffer):
        def __init__(self, num):
            super().__init__()
            self.test_field = num


    class TestSource(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.output = self.createOutput()

        def run(self):
            for i in range(10):
                buffer = MyCustomMessage(i)
                self.output.send(buffer)
                time.sleep(0.01)
    class TestSink(dai.node.HostNode):
        def __init__(self, input):
            dai.node.HostNode.__init__(self)
            self.count = 0
            self.link_args(input)

        def process(self, buffer):
            assert buffer.test_field == self.count
            self.count += 1
            if self.count == 10:
                self.stopPipeline()


    with dai.Pipeline(False) as p:
        source = TestSource()
        sink = TestSink(source.output)
        one_is_correct = False
        for n in p.getAllNodes():
            if isinstance(n, TestSink):
                one_is_correct = True
        assert one_is_correct
        p.start()
        p.wait()
        

test_with_host_nodes()
