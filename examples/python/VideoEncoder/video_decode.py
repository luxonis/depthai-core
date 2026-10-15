from datetime import timedelta

import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    camera = pipeline.create(dai.node.Camera).build()
    # Encode camera frames as MJPEG on the device before transferring them to the host.
    encoder = pipeline.create(dai.node.VideoEncoder)
    # Use H264_BASELINE, H264_MAIN or H264_HIGH in builds with OpenH264 support.
    encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.MJPEG)
    camera.requestOutput((640, 400), type=dai.ImgFrame.Type.NV12).link(encoder.input)

    # The encoded stream is decoded on the host using the SDK's OpenCV support.
    decoder = pipeline.create(dai.node.VideoDecoder)
    decoder.setOutFrameType(dai.ImgFrame.Type.BGR888i)
    encoder.out.link(decoder.input)
    frames = decoder.out.createOutputQueue()

    # Display decoded frames. Press q to exit.
    pipeline.start()
    while pipeline.isRunning():
        frame = frames.get(timeout=timedelta(seconds=1))
        if frame is not None:
            cv2.imshow("Decoded MJPEG", frame.getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
    pipeline.stop()
    pipeline.wait()

cv2.destroyAllWindows()
