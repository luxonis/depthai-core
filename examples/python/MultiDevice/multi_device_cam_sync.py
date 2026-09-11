#!/usr/bin/env python3
import cv2
import depthai as dai

dev1 = dai.Device()
dev2 = dai.Device()
with dai.Pipeline(createImplicitDevice=False) as pipeline:
    cam1 = pipeline.create(dai.node.Camera, dev1).build(dai.CameraBoardSocket.CAM_A)
    cam2 = pipeline.create(dai.node.Camera, dev2).build(dai.CameraBoardSocket.CAM_A)

    out1 = cam1.requestOutput((300, 300))
    out2 = cam2.requestOutput((300, 300))

    sync = pipeline.create(dai.node.Sync)
    sync.setRunOnHost(True)

    out1.link(sync.inputs["dev1"])
    out2.link(sync.inputs["dev2"])

    queue = sync.out.createOutputQueue()

    pipeline.start()

    while pipeline.isRunning():
        message_group = queue.get()
        frame1 = message_group["dev1"]
        frame2 = message_group["dev2"]

        img1 = frame1.getCvFrame()
        img2 = frame2.getCvFrame()

        combined = cv2.hconcat([img1, img2])
        cv2.imshow("multi_device_synced", combined)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
