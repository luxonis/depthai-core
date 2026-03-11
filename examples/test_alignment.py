
import cv2
import depthai as dai
import numpy as np


device = dai.Device()
fps = 30
with dai.Pipeline(device) as pipeline:

    camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    # if not rectified:
    camera_c_output = camera_c.requestOutput(
        (1280, 800), fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
    )
    camera_b_output = camera_b.requestOutput(
        (1280, 800), fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
    )

    reference_camera_output = camera_a.requestOutput(
        (1280, 800),
        fps=fps,
        enableUndistortion=True,
        resizeMode=dai.ImgResizeMode.CROP,
    )

    stereo = pipeline.create(dai.node.StereoDepth).build(camera_b_output, camera_c_output)
    reference_camera_output.link(stereo.inputAlignTo)

    cam_a_q = reference_camera_output.createOutputQueue()
    depth_q = stereo.depth.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        frame_a = cam_a_q.get()
        depth_frame = depth_q.get()
        
        assert isinstance(frame_a, dai.ImgFrame)
        assert isinstance(depth_frame, dai.ImgFrame)
        
        # get cv frame and overlay
        frame_a_img = frame_a.getCvFrame()
        depth_img = depth_frame.getCvFrame()
        depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
        overlay = cv2.addWeighted(frame_a_img, 0.7, depth_colored, 0.3, 0)
        cv2.imshow("Overlay", overlay)
        if cv2.waitKey(1) == ord("q"):
            pipeline.stop()
            break
        