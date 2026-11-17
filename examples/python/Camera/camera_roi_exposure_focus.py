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
    transformation = None
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
            start_points = None
            if roi_rect[2] == 0 or roi_rect[3] == 0:
                return
            print(f"ROI selected: {roi_rect}. Setting exposure and focus to this region.")
            # The ROI is in pixels of the shown frame; the frame's transformation maps it to the camera's sensor
            roi = dai.Rect(*roi_rect, False)
            ctrl = dai.CameraControl()
            ctrl.setAutoExposureRegion(roi, transformation)
            ctrl.setAutoFocusRegion(roi, transformation)
            cam_q_in.send(ctrl)

    # Create a window and set the mouse callback
    cv2.namedWindow("video")
    cv2.setMouseCallback("video", select_roi)

    while pipeline.isRunning():
        img_hd: dai.ImgFrame = stream_q.get()
        transformation = img_hd.getTransformation()
        frame = img_hd.getCvFrame()

        # Draw the ROI rectangle if it exists
        if roi_rect is not None:
            x, y, w, h = roi_rect
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow("video", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
