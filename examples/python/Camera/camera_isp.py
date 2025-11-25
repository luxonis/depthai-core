import cv2
import depthai as dai

# Create pipeline
with dai.Pipeline() as pipeline:
    # Define source and output
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    videoQueue = cam.requestOutput((800,400), fps=30).createOutputQueue()
    videoIsp = cam.requestIspOutput(fps=2).createOutputQueue()
    # Connect to device and start pipeline
    pipeline.start()
    videoIn = videoQueue.get()
    videoInIsp = videoIsp.get()
    print(
        "Standart output resolution = "
        f"{ videoIn.getCvFrame().shape[1]} x { videoIn.getCvFrame().shape[0]}"
    )
    print(
        f"Isp output resolution = "
        f"{ videoInIsp.getCvFrame().shape[1]} x { videoInIsp.getCvFrame().shape[0]}"
    )
    while pipeline.isRunning():
        videoIn = videoQueue.tryGet()
        videoInIsp = videoIsp.tryGet() # Returns 640x400
        if videoIn:
            cv2.imshow("video", videoIn.getCvFrame())
        if videoInIsp:
            cv2.imshow("videoIsp", videoInIsp.getCvFrame())

        if cv2.waitKey(1) == ord("q"):
            break
