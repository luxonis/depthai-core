"""Filesystem-path conversion accepts strings/Path and rejects unrelated objects."""
import os
from pathlib import Path

import depthai as dai
import pytest


@pytest.mark.parametrize("path_type", [str, Path])
def test_asset_path_conversion(tmp_path, path_type):
    path = tmp_path / "asset.bin"
    path.write_bytes(b"binding test")
    manager = dai.AssetManager()
    asset = manager.set("asset", path_type(path))
    assert bytes(asset.data) == b"binding test"


def test_asset_path_conversion_negative():
    with pytest.raises(TypeError):
        dai.AssetManager().set("invalid", dai.DeviceInfo())


@pytest.mark.skipif(os.environ.get("DEPTHAI_BINDINGS_DEVICE_TESTS") != "1", reason="Requires a DepthAI device")
@pytest.mark.parametrize("method_name", ["setBlobPath", "setBlob"])
def test_network_path_conversion(tmp_path, method_name):
    path = tmp_path / "missing.blob"
    with dai.Device() as device, dai.Pipeline(device) as pipeline:
        network = pipeline.create(dai.node.NeuralNetwork)
        method = getattr(network, method_name)
        for path_value in (str(path), path):
            with pytest.raises(RuntimeError):
                method(path_value)
        with pytest.raises(TypeError):
            method(dai.DeviceInfo())
