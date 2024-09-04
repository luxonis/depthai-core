import depthai as dai
import cv2

with dai.Pipeline(True) as pipeline:
    thermal = pipeline.create(dai.node.Thermal)
    thermal_img_out = thermal.color.createOutputQueue()

    pipeline.start()
    while True:
        thermal_img = thermal_img_out.get()
        cv2.imshow("thermal", thermal_img.getCvFrame())
        if cv2.waitKey(1) == ord("q"):
            break
