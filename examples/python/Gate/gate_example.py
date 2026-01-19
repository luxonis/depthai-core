import time
import cv2 
import depthai as dai

device = dai.Device()

withGate = True 

with dai.Pipeline(device) as pipeline:
    camera = pipeline.create(dai.node.Camera).build()
    cameraOut = camera.requestOutput((640, 400), fps=30)

    gate  = pipeline.create(dai.node.Gate)
    cameraOut.link(gate.input)
    cameraQueue = gate.output.createOutputQueue()
    gateControlQueue = gate.inputControl.createInputQueue()

    gateControl = dai.GateControl()

    pipeline.start()

    ellapsed = 0
    start_time = time.monotonic()
    gateOpen = True

    while pipeline.isRunning():
        frame = cameraQueue.tryGet()
        if frame is not None:
            cv2.imshow("camera", frame.getCvFrame())

        current_time = time.monotonic()
        elapsed = current_time - start_time

        if elapsed > 5.0:  # 5 seconds
            # Toggle the value
            gateOpen = not gateOpen
            # Create and send the control message
            if (gateOpen):
                gateControlQueue.send(dai.GateControl.openGate())
            else:
                gateControlQueue.send(dai.GateControl.closeGate())
            
            print(f"Gate toggled to: {gateOpen}")
            
            # Reset the timer
            start_time = current_time

        key = cv2.waitKey(1)

        if key == ord('q'):
            pipeline.stop()
            break
