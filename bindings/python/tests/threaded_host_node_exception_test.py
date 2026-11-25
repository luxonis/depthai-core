import depthai as dai


class FailingNode(dai.node.ThreadedHostNode):
    def run(self):
        raise ValueError("run() fails on purpose")


def test_python_exception_in_run_stops_pipeline():
    with dai.Pipeline(False) as pipeline:
        pipeline.create(FailingNode)
        pipeline.run()
        assert not pipeline.isRunning()
