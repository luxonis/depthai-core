import os
import sys
import time
import depthai as dai
import cv2 

deviceInfo = dai.DeviceInfo()
device = dai.Device(deviceInfo)

withGate = True 

with dai.Pipeline(device) as pipeline:
    camera = pipeline.create(dai.node.Camera).build()
    cameraOut = camera.requestFullResolutionOutput(fps=30)

    gate  = pipeline.create(dai.node.Gate)
    cameraOut.link(gate.input)
    cameraQueue = gate.output.createOutputQueue()
    gateControlQueue = gate.inputControl.createInputQueue()

    gateControl = dai.GateControl()
    gateControl.stop()

    pipeline.start()

    ellapsed = 0
    start_time = time.monotonic()
    gate_state = True

    while pipeline.isRunning():
        frame = cameraQueue.tryGet()
        if frame is not None:
            cv2.imshow("camera", frame.getCvFrame())

        current_time = time.monotonic()
        elapsed = current_time - start_time

        if elapsed > 5.0:  # 5 seconds
            # Toggle the value
            gate_state = not gate_state
            # Create and send the control message
            ctrl = dai.GateControl()
            ctrl.value = gate_state
            gateControlQueue.send(ctrl)
            
            print(f"Gate toggled to: {gate_state}")
            
            # Reset the timer
            start_time = current_time

        key = cv2.waitKey(1)

        if key == ord('q'):
            pipeline.stop()
            break
