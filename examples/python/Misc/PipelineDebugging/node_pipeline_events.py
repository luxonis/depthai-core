import depthai as dai
import numpy as np
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

    disparityQueue = stereo.disparity.createOutputQueue()
    monoLeftEventQueue = monoLeft.pipelineEventOutput.createOutputQueue()

    colorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
    colorMap[0] = [0, 0, 0]  # to make zero-disparity pixels black

    pipeline.start()
    maxDisparity = 1
    while pipeline.isRunning():
        disparity = disparityQueue.get()
        latestEvent = monoLeftEventQueue.tryGet()
        assert isinstance(disparity, dai.ImgFrame)
        npDisparity = disparity.getFrame()
        maxDisparity = max(maxDisparity, np.max(npDisparity))
        colorizedDisparity = cv2.applyColorMap(((npDisparity / maxDisparity) * 255).astype(np.uint8), colorMap)
        cv2.imshow("disparity", colorizedDisparity)
        print(f"Latest event from MonoLeft camera node: {latestEvent if latestEvent is not None else 'No event'}")
        key = cv2.waitKey(1)
        if key == ord('q'):
            pipeline.stop()
            break
