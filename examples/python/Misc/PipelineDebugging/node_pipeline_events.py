import time

import depthai as dai
import cv2

with dai.Pipeline() as pipeline:
    # Pipeline debugging is disabled by default.
    # You can also enable it by setting the DEPTHAI_PIPELINE_DEBUGGING environment variable to '1'
    pipeline.enablePipelineDebugging(True)

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    stereo = pipeline.create(dai.node.StereoDepth)

    # Linking
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)

    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    depthQueue = stereo.depth.createOutputQueue()
    monoLeftEventQueue = monoLeft.pipelineEventOutput.createOutputQueue() # Supported on core and rvc4 only

    pipeline.start()
    while pipeline.isRunning():
        depth = depthQueue.get()
        latestEvent = monoLeftEventQueue.tryGet()
        assert isinstance(depth, dai.ImgFrame)
        colorizedDepth = dai.colorizeDepthFrame(depth, 500, 12000, cv2.COLORMAP_JET, useLog=True).getCvFrame()
        cv2.imshow("depth", colorizedDepth)
        print(f"Latest event from MonoLeft camera node: {latestEvent if latestEvent is not None else 'No event'}")
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
