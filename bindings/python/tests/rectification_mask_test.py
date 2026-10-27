from datetime import timedelta
import unittest

import depthai as dai
import numpy as np


def make_frame(size, distortion, value, sequence, right=False):
    width, height = size
    frame = dai.ImgFrame()
    frame.setCvFrame(np.full((height, width), value, dtype=np.uint8), dai.ImgFrame.Type.RAW8)
    transform = dai.ImgTransformation(width, height)
    transform.setIntrinsicMatrix([[width * 0.7, 0, width / 2], [0, width * 0.7, height / 2], [0, 0, 1]])
    transform.setDistortionCoefficients([distortion, 0, 0, 0, 0])
    transform.setExtrinsics(dai.Extrinsics(
        [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        dai.Point3f(7.5 if right else 0, 0, 0),
        dai.CameraBoardSocket.CAM_B,
    ))
    frame.setTransformation(transform)
    frame.setSequenceNum(sequence)
    frame.setTimestamp(timedelta(seconds=sequence + 1))
    frame.setTimestampDevice(timedelta(seconds=sequence + 2))
    return frame


def check_usable_area_masks(output_size):
    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        rectification = pipeline.create(dai.node.Rectification)
        rectification.setRunOnHost(True)
        if output_size is not None:
            rectification.setOutputSize(*output_size)
        inputs = [rectification.input1.createInputQueue(), rectification.input2.createInputQueue()]
        images = [rectification.output1.createOutputQueue(), rectification.output2.createOutputQueue()]
        masks = [rectification.validMask1.createOutputQueue(), rectification.validMask2.createOutputQueue()]
        pipeline.start()
        first_masks = []
        latest_masks = [None, None]
        # Reusing the maps, changing calibration, then changing input dimensions must all work.
        for sequence, (size, distortion, value) in enumerate([
            ((128, 96), 0.7, 255),
            ((128, 96), 0.7, 0),
            ((128, 96), 0, 0),
            ((64, 48), 0.7, 255),
            ((64, 48), 0.7, 0),
        ]):
            for side in range(2):
                inputs[side].send(make_frame(size, distortion, value, sequence, right=bool(side)))
            for side in range(2):
                image = images[side].get(timeout=timedelta(seconds=5))
                assert image is not None
                if sequence in (1, 4):
                    # Changed pixels/timestamps must not trigger another mask message.
                    assert masks[side].get(timeout=timedelta(milliseconds=30)) is None
                    mask = latest_masks[side]
                else:
                    mask = masks[side].get(timeout=timedelta(seconds=5))
                    assert mask is not None
                    latest_masks[side] = mask
                    assert mask.getSequenceNum() == image.getSequenceNum() == sequence
                    assert mask.getTimestamp() == image.getTimestamp()
                    assert mask.getTimestampDevice() == image.getTimestampDevice()
                pixels, valid = image.getCvFrame(), mask.getCvFrame()
                assert mask.getType() == dai.ImgFrame.Type.RAW8
                assert valid.shape == pixels.shape
                assert valid.shape == (output_size or size)[::-1]
                assert mask.getTransformation().isEqualTransformation(image.getTransformation())
                assert np.all((valid == 0) | (valid == 255))
                assert np.any(valid == 255)
                if distortion:
                    assert np.any(valid == 0)
                if value == 255:
                    assert np.all(pixels[valid == 255] == 255)
                    assert np.all(valid[pixels < 255] == 0)
                    partial = (pixels > 0) & (pixels < 255)
                    assert np.any(partial)
                    assert np.all(valid[partial] == 0)
                else:
                    # Black scene pixels are still usable; image thresholding cannot produce this mask.
                    assert np.all(pixels == 0)
                if sequence == 0:
                    first_masks.append(valid.copy())
                elif sequence == 1:
                    np.testing.assert_array_equal(valid, first_masks[side])
                elif sequence == 2:
                    assert np.count_nonzero(valid) > np.count_nonzero(first_masks[side])


def check_optional_masks(connected_mask):
    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        rectification = pipeline.create(dai.node.Rectification)
        rectification.setRunOnHost(True)
        inputs = [rectification.input1.createInputQueue(), rectification.input2.createInputQueue()]
        outputs = [rectification.output1.createOutputQueue(), rectification.output2.createOutputQueue()]
        if connected_mask is not None:
            outputs.append(getattr(rectification, f"validMask{connected_mask}").createOutputQueue())
        pipeline.start()
        for side in range(2):
            inputs[side].send(make_frame((64, 48), 0, 0, 0, right=bool(side)))
        for queue in outputs:
            assert queue.get(timeout=timedelta(seconds=5)) is not None


class TestRectificationMasks(unittest.TestCase):
    def test_usable_area_masks(self):
        for output_size in (None, (96, 72)):
            with self.subTest(output_size=output_size):
                check_usable_area_masks(output_size)

    def test_optional_masks(self):
        for connected_mask in (None, 1, 2):
            with self.subTest(connected_mask=connected_mask):
                check_optional_masks(connected_mask)
