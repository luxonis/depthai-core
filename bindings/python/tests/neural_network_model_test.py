"""Host-only tests for loading neural network models from memory and paths."""

import pytest

from depthai_pybind11_tests import neural_network_model as m


@pytest.mark.parametrize("model_type", [bytes, bytearray, list])
@pytest.mark.parametrize("data", [b"", bytes(range(256))])
def test_set_other_model_format_from_memory(model_type, data):
    nn = m.create_neural_network()

    nn.setOtherModelFormat(model_type(data))

    assert bytes(nn.getAssetManager().get("__model").data) == data


def test_set_other_model_format_owns_model_data():
    nn = m.create_neural_network()
    data = bytes(range(256))
    model = bytearray(data)

    nn.setOtherModelFormat(model=model)
    model[:] = b"\x00" * len(model)
    del model

    assert bytes(nn.getAssetManager().get("__model").data) == data


@pytest.mark.parametrize("string_path", [False, True])
@pytest.mark.parametrize("keyword_path", [False, True])
def test_set_other_model_format_from_path(tmp_path, string_path, keyword_path):
    nn = m.create_neural_network()
    data = bytes(range(256))
    path = tmp_path / "model.dlc"
    path.write_bytes(data)

    path = str(path) if string_path else path
    if keyword_path:
        nn.setOtherModelFormat(path=path)
    else:
        nn.setOtherModelFormat(path)

    assert bytes(nn.getAssetManager().get("__model").data) == data
