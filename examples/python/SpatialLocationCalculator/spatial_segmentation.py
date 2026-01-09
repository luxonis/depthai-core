import depthai as dai
import numpy as np
import cv2

modelName = "luxonis/yolov8-instance-segmentation-large:coco-640x480"
setRunOnHost = False
device = dai.Device()
fps = 30
if device.getPlatform() == dai.Platform.RVC2:
    modelName = "luxonis/yolov8-instance-segmentation-nano:coco-512x288"
    setRunOnHost = True
    fps = 10

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    detNN = pipeline.create(dai.node.DetectionNetwork).build(cameraNode, dai.NNModelDescription(modelName), fps=fps)
    detNN.detectionParser.setRunOnHost(setRunOnHost)

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    align = pipeline.create(dai.node.ImageAlign)
    stereo.depth.link(align.input)
    detNN.passthrough.link(align.inputAlignTo)

    spatialCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
    spatialCalculator.initialConfig.setUseSegmentation(True)
    align.outputAligned.link(spatialCalculator.inputDepth)
    detNN.out.link(spatialCalculator.inputDetections)

    cameraQueue = detNN.passthrough.createOutputQueue()
    spatialOutputQueue = spatialCalculator.outputDetections.createOutputQueue()
    depthQueue = spatialCalculator.passthroughDepth.createOutputQueue()
    inputConfigQueue = spatialCalculator.inputConfig.createInputQueue()
    calculatorConfig = spatialCalculator.initialConfig # copy initial settings

    pipeline.start()
    print("Pipeline created.")
    while pipeline.isRunning():
        spatialDetections = spatialOutputQueue.get()
        rgbFrame = cameraQueue.get()
        depthFrame = depthQueue.get()

        assert isinstance(spatialDetections, dai.SpatialImgDetections)
        assert isinstance(rgbFrame, dai.ImgFrame)
        assert isinstance(depthFrame, dai.ImgFrame)

        depthImg = depthFrame.getCvFrame()
        colorizedDepth = cv2.applyColorMap(cv2.convertScaleAbs(depthImg, alpha=0.03), cv2.COLORMAP_JET)
        image = rgbFrame.getCvFrame()
        segmentationMask = cv2.Mat(np.zeros((spatialDetections.getSegmentationMaskHeight(), spatialDetections.getSegmentationMaskWidth()), dtype=np.uint8))
        segmentationMask = spatialDetections.getCvSegmentationMask()

        if segmentationMask is not None and calculatorConfig.getUseSegmentation():
            scaledMask = segmentationMask.copy()
            scaledMask[segmentationMask != 255] = segmentationMask[segmentationMask != 255] * 25 # scale for better visualization
            coloredMask = cv2.applyColorMap(scaledMask, cv2.COLORMAP_JET)
            coloredMask[segmentationMask == 255] = image[segmentationMask == 255]
            image = cv2.addWeighted(image, 0.7, coloredMask, 0.3, 0)

        for i, det in enumerate(spatialDetections.detections):
            outerPoints = det.getBoundingBox()
            if outerPoints is not None:
                outerPoints = outerPoints.denormalize(image.shape[1], image.shape[0])
                outerPoints = outerPoints.getPoints()
                outerPoints = [[int(p.x), int(p.y)] for p in outerPoints]
                outerPoints = np.array(outerPoints, dtype=np.int32)
                cv2.polylines(image, [outerPoints], isClosed=True, color=(0, 255, 0), thickness=2)

                # depth
                depth_coordinate = det.spatialCoordinates
                depth = depth_coordinate.z
                text = f"X: {int(depth_coordinate.x / 10 )} cm, Y: {int(depth_coordinate.y / 10)} cm, Z: {int(depth / 10)} cm"
                cv2.putText(image, text, outerPoints[0], cv2.FONT_HERSHEY_PLAIN, 1, (232,36,87), 2)

        topPanel = np.ones((100, image.shape[1], 3), dtype=np.uint8) * 255
        cv2.putText(topPanel, "Press 's' to toggle setUseSegmentation", (10, 30), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)
        cv2.putText(topPanel, f"Current setUseSegmentation: {calculatorConfig.getUseSegmentation()}", (10, 60), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)
        concatenatedFrame = cv2.vconcat([topPanel, image])

        cv2.imshow("Depth", colorizedDepth)
        cv2.imshow("Spatial detections", concatenatedFrame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        elif key == ord('s'):
            calculatorConfig.setUseSegmentation(not calculatorConfig.getUseSegmentation())
            # calculatorConfig.setSegmentationPassthrough(not calculatorConfig.getSegmentationPassthrough())
            inputConfigQueue.send(calculatorConfig)