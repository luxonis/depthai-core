"""Host-only tests for loading neural network models from memory and paths."""

import depthai as dai
import pytest


@pytest.mark.parametrize("model_type", [bytes, bytearray, list])
@pytest.mark.parametrize("data", [b"", bytes(range(256))])
def test_set_other_model_format_from_memory(model_type, data):
    pipeline = dai.Pipeline(False)
    nn = pipeline.create(dai.node.NeuralNetwork)

    nn.setOtherModelFormat(model_type(data))

    assert bytes(nn.getAssetManager().get("__model").data) == data


def test_set_other_model_format_owns_model_data():
    pipeline = dai.Pipeline(False)
    nn = pipeline.create(dai.node.NeuralNetwork)
    data = bytes(range(256))
    model = bytearray(data)

    nn.setOtherModelFormat(model=model)
    model[:] = b"\x00" * len(model)
    del model

    assert bytes(nn.getAssetManager().get("__model").data) == data


@pytest.mark.parametrize("string_path", [False, True])
def test_set_other_model_format_from_path(tmp_path, string_path):
    pipeline = dai.Pipeline(False)
    nn = pipeline.create(dai.node.NeuralNetwork)
    data = bytes(range(256))
    path = tmp_path / "model.dlc"
    path.write_bytes(data)

    nn.setOtherModelFormat(path=str(path) if string_path else path)

    assert bytes(nn.getAssetManager().get("__model").data) == data
