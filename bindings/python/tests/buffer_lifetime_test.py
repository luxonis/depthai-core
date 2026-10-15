import gc

import depthai as dai
import numpy as np
import pytest


@pytest.mark.parametrize("as_list", [False, True])
def test_data_view_survives_storage_replacement(as_list):
    buffer = dai.Buffer()
    expected = np.arange(128, dtype=np.uint8)
    buffer.setData(expected)
    view = buffer.getData()
    replacement = np.full(256, 255, dtype=np.uint8)
    buffer.setData(replacement.tolist() if as_list else replacement)

    np.testing.assert_array_equal(view, expected)
    view[:] = 42
    np.testing.assert_array_equal(buffer.getData(), replacement)

    del buffer
    gc.collect()
    np.testing.assert_array_equal(view, np.full(128, 42, dtype=np.uint8))


def test_data_views_share_storage_and_outlive_buffer():
    buffer = dai.Buffer()
    buffer.setData(np.arange(16, dtype=np.uint8))
    view = buffer.getData()
    view[3] = 99
    assert buffer.getData()[3] == 99
    sliced = view[::2]
    del view, buffer
    gc.collect()
    np.testing.assert_array_equal(sliced, np.arange(0, 16, 2, dtype=np.uint8))


@pytest.mark.parametrize("replacement", [[9, 8], b"\x09\x08", np.array([9, 8], dtype=np.uint8)])
def test_set_data_shrinks_storage_and_preserves_previous_view(replacement):
    buffer = dai.Buffer()
    buffer.setData(np.arange(128, dtype=np.uint8))
    previous = buffer.getData()
    buffer.setData(replacement)
    np.testing.assert_array_equal(buffer.getData(), [9, 8])
    np.testing.assert_array_equal(previous, np.arange(128, dtype=np.uint8))


@pytest.mark.parametrize("source", [
    np.arange(16, dtype=np.uint8)[::-2],
    np.arange(8, dtype=np.uint16),
    memoryview(bytearray(range(16)))[::2],
    bytearray(range(8)),
])
def test_set_data_accepts_strided_and_convertible_buffers(source):
    buffer = dai.Buffer()
    buffer.setData(source)
    np.testing.assert_array_equal(buffer.getData(), np.asarray(source, dtype=np.uint8))


@pytest.mark.parametrize("mode", ["inferred", "datatype", "storage_order"])
def test_nndata_view_survives_adding_tensor(mode):
    message = dai.NNData()
    message.addTensor("first", np.arange(16, dtype=np.uint8))
    previous = message.getData()
    expected = previous.copy()
    tensor = np.arange(1024, dtype=np.int32) % 256
    if mode == "inferred":
        message.addTensor("second", tensor)
    elif mode == "datatype":
        message.addTensor("second", tensor, dai.TensorInfo.DataType.INT)
    else:
        message.addTensor("second", tensor.tolist(), dai.TensorInfo.StorageOrder.NC)
    np.testing.assert_array_equal(previous, expected)
    np.testing.assert_array_equal(message.getTensor("second").ravel(), tensor)


@pytest.mark.parametrize("stride", [32, 40])
def test_mask_view_survives_frame_replacement(stride):
    mask = dai.SegmentationMask()
    previous = mask.prepareMask(4, 4)
    previous[:] = np.arange(16, dtype=np.uint8)
    frame = dai.ImgFrame()
    frame.setType(dai.ImgFrame.Type.GRAY8)
    frame.setSize(32, 24)
    frame.setStride(stride)
    frame.setData(np.full(stride * 24, 7, dtype=np.uint8))
    mask.setMask(frame)
    np.testing.assert_array_equal(previous, np.arange(16, dtype=np.uint8))
    assert (mask.getWidth(), mask.getHeight()) == (32, 24)
    np.testing.assert_array_equal(mask.getMaskData(), np.full((24, 32), 7, dtype=np.uint8))


@pytest.mark.skipif(not hasattr(dai, "beta"), reason="Requires beta bindings")
def test_map_view_survives_storage_growth():
    message = dai.beta.Map2D()
    message.setMap(np.arange(16, dtype=np.float32).reshape(4, 4))
    previous = message.getData()
    expected = previous.copy()
    message.setMap(np.full((32, 32), 7, dtype=np.float32))
    np.testing.assert_array_equal(previous, expected)
    np.testing.assert_array_equal(message.getMap(), np.full((32, 32), 7, dtype=np.float32))
