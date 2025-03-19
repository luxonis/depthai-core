#!/usr/bin/env python3

import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cam_input_q = cam.inputControl.createInputQueue()
    stream_q = cam.requestOutput((1920, 1080)).createOutputQueue()

    cam_q_in = cam.inputControl.createInputQueue()

    # Connect to device and start pipeline
    pipeline.start()

    # ROI selection variables
    start_points = []
    roi_rect = None
    scale_factors = None
    # Mouse callback function for ROI selection
    def select_roi(event, x, y, flags, param):
        global start_points, roi_rect
        def set_roi_rect():
            global roi_rect
            x1, y1 = start_points
            x2, y2 = (x, y)
            roi_rect = (min(x1, x2), min(y1, y2), abs(x2-x1), abs(y2-y1))

        if event == cv2.EVENT_LBUTTONDOWN:
            roi_rect = None
            start_points = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and start_points:
            set_roi_rect()
        elif event == cv2.EVENT_LBUTTONUP and start_points:
            set_roi_rect()
            roi_rect_scaled = (
                int(roi_rect[0] * scale_factors[0]),
                int(roi_rect[1] * scale_factors[1]),
                int(roi_rect[2] * scale_factors[0]),
                int(roi_rect[3] * scale_factors[1])
            )
            print(f"ROI selected: {roi_rect}")
            ctrl = dai.CameraControl()
            print(f"Scaled ROI selected: {roi_rect_scaled}. Setting exposure and focus to this region.")
            ctrl.setAutoExposureRegion(*roi_rect_scaled)
            ctrl.setAutoFocusRegion(*roi_rect_scaled)
            cam_q_in.send(ctrl)
            start_points = None

    # Create a window and set the mouse callback
    cv2.namedWindow("video")
    cv2.setMouseCallback("video", select_roi)

    while pipeline.isRunning():
        img_hd: dai.ImgFrame = stream_q.get()
        if scale_factors is None:
            print(img_hd.getTransformation().getSourceSize(), img_hd.getTransformation().getSize())
            scale_factors = (img_hd.getTransformation().getSourceSize()[0] / img_hd.getTransformation().getSize()[0],
                            img_hd.getTransformation().getSourceSize()[1] / img_hd.getTransformation().getSize()[1])
        frame = img_hd.getCvFrame()

        # Draw the ROI rectangle if it exists
        if roi_rect is not None:
            x, y, w, h = roi_rect
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow("video", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
