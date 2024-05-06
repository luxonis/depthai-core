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
    dai.SystemInformationS3,
    dai.ToFConfig,
    dai.TrackedFeatures,
    dai.Tracklets
]

def test_constructable():
    for message_type in message_types:
        message = message_type()
        assert message is not None


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

def test_with_host_nodes():
    import time
    class MyCustomMessage(dai.Buffer):
        def __init__(self, num):
            super().__init__()
            self.test_field = num


    class TestSource(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.output = dai.Node.Output(self)

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