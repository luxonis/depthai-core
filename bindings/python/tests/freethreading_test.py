"""Hardware-independent checks for parallel Python pipeline contexts."""

from concurrent.futures import ThreadPoolExecutor
import os
import subprocess
import sys
import sysconfig
import threading

import depthai as dai
import depthai_pybind11_tests as native_tests
import pytest


if not native_tests.gil_free_build:
    pytest.skip("Requires DEPTHAI_PYTHON_GIL_FREE=ON", allow_module_level=True)


@pytest.mark.skipif(sysconfig.get_config_var("Py_GIL_DISABLED") != 1, reason="Requires free-threaded Python")
def test_imports_do_not_enable_gil():
    # Do not force gil=0 here: detect a dependency that would silently enable it.
    environment = dict(os.environ)
    environment.pop("PYTHON_GIL", None)
    script = """
import sys
assert not sys._is_gil_enabled()
import numpy, depthai, depthai_pybind11_tests
assert not sys._is_gil_enabled(), "An imported extension enabled the GIL"
"""
    result = subprocess.run(
        [sys.executable, "-Werror", "-c", script],
        env=environment, capture_output=True, text=True, timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_parallel_nested_pipeline_contexts():
    barrier = threading.Barrier(8, timeout=15)

    def create_nodes(_):
        for _ in range(10):
            with dai.Pipeline(False) as outer:
                barrier.wait()
                first = outer.create(dai.node.MessageDemux)
                with dai.Pipeline(False) as inner:
                    second = inner.create(dai.node.MessageDemux)
                    assert second.getParentPipeline().getAllNodes() == [second]
                    assert inner.getAllNodes() == [second]
                third = outer.create(dai.node.MessageDemux)
                assert first in first.getParentPipeline().getAllNodes()
                assert third in third.getParentPipeline().getAllNodes()
                assert second not in outer.getAllNodes()
                assert first in outer.getAllNodes() and third in outer.getAllNodes()

    with ThreadPoolExecutor(max_workers=8) as pool:
        list(pool.map(create_nodes, range(8)))


@pytest.mark.parametrize("message_type", [
    dai.ImgFrame, dai.NNData, dai.IMUData, dai.PointCloudData,
    dai.ImgDetections, dai.SpatialImgDetections, dai.SegmentationMask,
    dai.ImgAnnotations, dai.EncodedFrame, dai.RGBDData,
])
def test_proto_serializable_shared_ownership_keeps_python_subclass(message_type):
    import gc

    class CustomMessage(message_type):
        def __init__(self):
            super().__init__()
            self.test_field = 42

    message = CustomMessage()
    message.setSequenceNum(123)
    retained = native_tests.retain_proto_serializable(message)
    del message
    gc.collect()

    restored = retained()
    assert isinstance(restored, CustomMessage)
    assert isinstance(restored, dai.ProtoSerializable)
    assert restored.test_field == 42
    assert restored.getSequenceNum() == 123
