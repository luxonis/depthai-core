import cv2
import depthai as dai
import numpy as np
import pytest
import numpy.typing as npt

DEBUG = False


def generate_color_image(width=96, height=72, dtype=np.uint8):
    y = np.arange(height, dtype=np.float32)[:, None]
    x = np.arange(width, dtype=np.float32)
    grid_x, grid_y = np.meshgrid(x, y)

    norm_x = grid_x / max(width - 1, 1)
    norm_y = grid_y / max(height - 1, 1)

    b = (0.6 * norm_x + 0.4 * norm_y) * 255.0
    g = (0.3 * norm_x + 0.7 * norm_y) * 255.0
    r = (0.8 * norm_x + 0.2 * norm_y) * 255.0

    image = np.stack([b, g, r], axis=-1)
    return np.clip(image, 0, 255).astype(dtype)


def generate_gray_image(width=64, height=48, dtype: npt.DTypeLike = np.uint8, max_value=None):
    y = np.arange(height, dtype=np.int64)[:, None]
    x = np.arange(width, dtype=np.int64)
    grid = x + 5 * y
    if max_value is not None:
        grid = np.mod(grid, max_value)
    return grid.astype(dtype)


def max_abs_diff(expected, recovered):
    diff = np.abs(expected.astype(np.float32) - recovered.astype(np.float32))
    if diff.ndim == 3:
        diff = diff.max(axis=2)
    return float(diff.max())


def assert_images_close(expected, recovered, tolerance, msg):
    max_diff = max_abs_diff(expected, recovered)
    assert max_diff <= tolerance, f"{msg} max abs diff too high: {max_diff} > {tolerance}"



COLOR_TYPES = [
    pytest.param(dai.ImgFrame.Type.BGR888p, 0.5, id="BGR888p"),
    pytest.param(dai.ImgFrame.Type.RGB888p, 0.5, id="RGB888p"),
    pytest.param(dai.ImgFrame.Type.BGR888i, 0.5, id="BGR888i"),
    pytest.param(dai.ImgFrame.Type.RGB888i, 0.5, id="RGB888i"),
    pytest.param(dai.ImgFrame.Type.RGB161616, 0.5, id="RGB161616"),
    pytest.param(dai.ImgFrame.Type.RGBF16F16F16i, 0.5, id="RGBF16F16F16i"),
    pytest.param(dai.ImgFrame.Type.BGRF16F16F16i, 0.5, id="BGRF16F16F16i"),
    pytest.param(dai.ImgFrame.Type.RGBF16F16F16p, 0.5, id="RGBF16F16F16p"),
    pytest.param(dai.ImgFrame.Type.BGRF16F16F16p, 0.5, id="BGRF16F16F16p"),
    pytest.param(dai.ImgFrame.Type.YUV420p, 4.0, id="YUV420p"),
    pytest.param(dai.ImgFrame.Type.NV12, 4.0, id="NV12"),
    pytest.param(dai.ImgFrame.Type.NV21, 4.0, id="NV21"),
]


@pytest.mark.parametrize("frame_type,tolerance", COLOR_TYPES)
def test_setcvframe_color_types(frame_type, tolerance):
    image = generate_color_image()
    frame = dai.ImgFrame()
    frame.setCvFrame(image, frame_type)

    recovered = frame.getCvFrame()

    assert recovered.shape[0] == image.shape[0]
    assert recovered.shape[1] == image.shape[1]
    assert recovered.shape[2] == 3

    expected = image
    if frame_type == dai.ImgFrame.Type.RGB161616:
        expected = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    assert_images_close(expected, recovered, tolerance, frame_type.name)


RAW_TYPES = [
    pytest.param(dai.ImgFrame.Type.RAW8, np.uint8, id="RAW8"),
    pytest.param(dai.ImgFrame.Type.GRAY8, np.uint8, id="GRAY8"),
    pytest.param(dai.ImgFrame.Type.RAW10, np.uint16, id="RAW10"),
    pytest.param(dai.ImgFrame.Type.RAW12, np.uint16, id="RAW12"),
    pytest.param(dai.ImgFrame.Type.RAW14, np.uint16, id="RAW14"),
    pytest.param(dai.ImgFrame.Type.RAW16, np.uint16, id="RAW16"),
]


@pytest.mark.parametrize("frame_type,expected_dtype", RAW_TYPES)
def test_setcvframe_raw_and_gray(frame_type, expected_dtype):
    image = generate_color_image()
    frame = dai.ImgFrame()
    frame.setCvFrame(image, frame_type)

    recovered = frame.getCvFrame()
    expected = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(expected_dtype)

    assert recovered.shape == expected.shape
    assert recovered.dtype == expected_dtype
    assert_images_close(expected, recovered, tolerance=0.5, msg=frame_type.name)


def test_setcvframe_grayf16():
    image = generate_gray_image(dtype=np.float32)
    frame = dai.ImgFrame()
    frame.setCvFrame(image, dai.ImgFrame.Type.GRAYF16)

    recovered = frame.getCvFrame()
    expected = image.astype(np.float16)

    assert recovered.shape == expected.shape
    assert recovered.dtype == np.float16
    assert_images_close(expected, recovered, tolerance=1e-3, msg="GRAYF16")


def test_setcvframe_raw32():
    image = generate_gray_image(width=40, height=30, dtype=np.int32, max_value=50000)
    frame = dai.ImgFrame()
    frame.setCvFrame(image, dai.ImgFrame.Type.RAW32)

    recovered = frame.getCvFrame()

    assert recovered.shape == image.shape
    assert recovered.dtype == np.int32
    assert_images_close(image, recovered, tolerance=0.0, msg="RAW32")
