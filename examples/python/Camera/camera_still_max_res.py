#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    cam_q_in = cam.inputControl.createInputQueue()

    cam_input_q = cam.inputControl.createInputQueue()
    # In some cases (IMX586), this requires an 8k screen to be able to see the full resolution at once
    stream_highest_res = cam.requestFullResolutionOutput(useHighestResolution=True)

    script = pipeline.create(dai.node.Script)
    stream_highest_res.link(script.inputs["in"])
    # Current workaround for OAK4 cameras, as Camera node doesn't yet support "still" frame capture
    script.setScript(
        """
        while True:
            message = node.inputs["in"].get()
            trigger = node.inputs["trigger"].tryGet()
            if trigger is not None:
                node.warn("Trigger received!")
                node.io["highest_res"].send(message)
        """)

    # If 8k, we can only have 1 output stream, so we need to use ImageManip to downscale
    imgManip = pipeline.create(dai.node.ImageManip)
    stream_highest_res.link(imgManip.inputImage)
    imgManip.initialConfig.setOutputSize(1333, 1000)
    imgManip.setMaxOutputFrameSize(1333*1000*3)
    downscaled_res_q = imgManip.out.createOutputQueue()

    highest_res_q = script.outputs["highest_res"].createOutputQueue()
    q_trigger = script.inputs["trigger"].createInputQueue()

    # Connect to device and start pipeline
    ctrl = dai.CameraControl()
    pipeline.start()
    print("To capture an image, press 'c'")
    while pipeline.isRunning():
        img_hd: dai.ImgFrame = downscaled_res_q.get()
        frame = img_hd.getCvFrame()
        cv2.imshow("video", frame)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        if key == ord('c'):
            # Send a trigger message to the Script node
            q_trigger.send(dai.Buffer())

        if highest_res_q.has():
            highres_img = highest_res_q.get()
            frame = highres_img.getCvFrame()
            # Save the full image
            cv2.imwrite("full_image.png", frame)
