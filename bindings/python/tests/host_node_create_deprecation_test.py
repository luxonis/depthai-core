import warnings

import depthai as dai
import pytest


class DummyThreadedHostNode(dai.node.ThreadedHostNode):
    def __init__(self):
        super().__init__()
        self.out = self.createOutput()

    def run(self):
        while self.mainLoop():
            pass


class DummyHostNode(dai.node.HostNode):
    def process(self):
        return dai.NNData()


def _pipeline_create_deprecation_warnings(caught_warnings):
    return [
        warning
        for warning in caught_warnings
        if warning.category is DeprecationWarning
        and "pipeline.create" in str(warning.message)
    ]


@pytest.mark.parametrize("node_type", [DummyThreadedHostNode, DummyHostNode])
def test_direct_host_node_construction_warns(node_type):
    with dai.Pipeline(False):
        with warnings.catch_warnings(record=True) as caught_warnings:
            warnings.simplefilter("always", DeprecationWarning)
            node_type()

    assert len(_pipeline_create_deprecation_warnings(caught_warnings)) == 1


@pytest.mark.parametrize("node_type", [DummyThreadedHostNode, DummyHostNode])
def test_pipeline_create_host_node_has_no_deprecation(node_type):
    with dai.Pipeline(False) as pipeline:
        with warnings.catch_warnings(record=True) as caught_warnings:
            warnings.simplefilter("always", DeprecationWarning)
            pipeline.create(node_type)

    assert len(_pipeline_create_deprecation_warnings(caught_warnings)) == 0
