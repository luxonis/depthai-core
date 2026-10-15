from datetime import timedelta

import depthai as dai
import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytestmark = pytest.mark.skipif(
    not hasattr(dai.node, "VideoDecoder"), reason="VideoDecoder requires OpenCV support"
)


def test_video_decoder_mjpeg():
    image = np.full((24, 32, 3), (30, 90, 180), dtype=np.uint8)
    success, jpeg = cv2.imencode(".jpg", image)
    assert success

    with dai.Pipeline(False) as pipeline:
        decoder = pipeline.create(dai.node.VideoDecoder)
        assert decoder.getOutFrameType() == dai.ImgFrame.Type.BGR888i
        assert decoder.setOutFrameType(dai.ImgFrame.Type.RGB888p) is decoder
        with pytest.raises(ValueError):
            decoder.setOutFrameType(dai.ImgFrame.Type.BITSTREAM)
        input_queue = decoder.input.createInputQueue()
        output_queue = decoder.out.createOutputQueue()

        encoded = dai.EncodedFrame()
        encoded.setProfile(dai.EncodedFrame.Profile.JPEG)
        encoded.setSize(32, 24)
        encoded.setSequenceNum(42)
        encoded.setData(jpeg)
        pipeline.start()
        input_queue.send(encoded)
        decoded = output_queue.get(timeout=timedelta(seconds=5))
        assert isinstance(decoded, dai.ImgFrame)
        assert decoded.getType() == dai.ImgFrame.Type.RGB888p
        assert decoded.getSequenceNum() == 42
        assert decoded.getWidth() == 32
        assert decoded.getHeight() == 24
        np.testing.assert_allclose(decoded.getCvFrame().mean(axis=(0, 1)), (30, 90, 180), atol=4)
        pipeline.stop()
        pipeline.wait()
