import warnings

import depthai as dai


class DummyThreadedHostNode(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.out = self.createOutput()

    def run(self):
        while self.mainLoop():
            pass


def _pipeline_create_deprecation_warnings(caught_warnings):
    return [
        warning
        for warning in caught_warnings
        if warning.category is DeprecationWarning
        and "pipeline.create" in str(warning.message)
    ]


def test_direct_threaded_host_node_construction_warns():
    with dai.Pipeline(False):
        with warnings.catch_warnings(record=True) as caught_warnings:
            warnings.simplefilter("always", DeprecationWarning)
            DummyThreadedHostNode()

    assert len(_pipeline_create_deprecation_warnings(caught_warnings)) == 1


def test_pipeline_create_threaded_host_node_has_no_deprecation():
    with dai.Pipeline(False) as pipeline:
        with warnings.catch_warnings(record=True) as caught_warnings:
            warnings.simplefilter("always", DeprecationWarning)
            pipeline.create(DummyThreadedHostNode)

    assert len(_pipeline_create_deprecation_warnings(caught_warnings)) == 0
