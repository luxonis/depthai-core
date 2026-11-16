import depthai as dai
import numpy as np
import pytest


@pytest.mark.parametrize(
    "datatype, values, expected_dtype",
    [
        (dai.TensorInfo.DataType.INT, [[16777217, -16777217], [2147483647, -2147483648]], np.int32),
        (dai.TensorInfo.DataType.I8, [[-128, -1], [0, 127]], np.int32),
        (dai.TensorInfo.DataType.U8F, [[0, 1], [128, 255]], np.int32),
        (dai.TensorInfo.DataType.FP64, [[1.000000000001, -1.000000000001], [1e100, 1e-100]], np.float64),
    ],
)
def test_all_tensor_getters_preserve_precision(datatype, values, expected_dtype):
    message = dai.NNData()
    expected = np.array(values, dtype=expected_dtype)
    message.addTensor("values", expected, datatype)
    order = dai.TensorInfo.StorageOrder.CN

    for actual, wanted in [
        (message.getTensor("values"), expected),
        (message.getFirstTensor(), expected),
        (message.getTensor("values", order), expected.T),
        (message.getFirstTensor(order), expected.T),
    ]:
        assert actual.dtype == np.dtype(expected_dtype)
        np.testing.assert_array_equal(actual, wanted)


@pytest.mark.parametrize("datatype", [dai.TensorInfo.DataType.I8, dai.TensorInfo.DataType.INT])
def test_dequantized_integer_getters_still_return_floats(datatype):
    message = dai.NNData()
    expected = np.array([[-2, -1], [0, 1]], dtype=np.int32)
    message.addTensor("values", expected, datatype)
    order = dai.TensorInfo.StorageOrder.CN

    for actual, wanted in [
        (message.getTensor("values", dequantize=True), expected),
        (message.getFirstTensor(dequantize=True), expected),
        (message.getTensor("values", order, dequantize=True), expected.T),
        (message.getFirstTensor(order, dequantize=True), expected.T),
    ]:
        assert np.issubdtype(actual.dtype, np.floating)
        np.testing.assert_array_equal(actual, wanted)
