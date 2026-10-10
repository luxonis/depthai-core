# -*- coding: utf-8 -*-
"""Host-only: pipeline.create(NodeClass, device) validates its device argument."""
import pytest

import depthai as dai


def test_pipeline_create_validates_device_argument():
    p = dai.Pipeline(False)
    # None means "no explicit device", same as omitting it
    assert isinstance(p.create(dai.node.Sync), dai.node.Sync)
    assert isinstance(p.create(dai.node.Sync, None), dai.node.Sync)
    assert isinstance(p.create(dai.node.Sync, device=None), dai.node.Sync)

    # Anything that is not a dai.Device raises TypeError, positionally and as keyword
    for bad in (dai.DeviceInfo("10.12.233.5"), "10.12.233.5", 1):
        with pytest.raises(TypeError, match=r"must be a depthai\.Device"):
            p.create(dai.node.Sync, bad)
        with pytest.raises(TypeError, match=r"must be a depthai\.Device"):
            p.create(dai.node.Sync, device=bad)
    # DeviceInfo / str get the addDevice(...) hint
    with pytest.raises(TypeError, match="addDevice"):
        p.create(dai.node.Sync, dai.DeviceInfo("10.12.233.5"))
    with pytest.raises(TypeError, match="addDevice"):
        p.create(dai.node.Sync, "10.12.233.5")

    # Call-shape errors are raised regardless of the device argument's type
    with pytest.raises(TypeError, match="at most one positional argument"):
        p.create(dai.node.Sync, 1, 2)
    with pytest.raises(TypeError, match="unexpected keyword argument 'foo'"):
        p.create(dai.node.Sync, foo=1)
    with pytest.raises(TypeError, match="multiple values for argument 'device'"):
        p.create(dai.node.Sync, None, device=None)


def test_pipeline_create_forwards_arguments_to_python_host_nodes():
    p = dai.Pipeline(False)

    class MyNode(dai.node.ThreadedHostNode):
        def __init__(self, value):
            super().__init__()
            self.value = value

        def run(self):
            pass

    assert p.create(MyNode, 42).value == 42
