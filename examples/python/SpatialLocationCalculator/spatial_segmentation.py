import depthai as dai
import numpy as np
import cv2

model_name = "luxonis/yolov8-instance-segmentation-large:coco-640x480"
setRunOnHost = False
device = dai.Device()
fps = 30
if device.getPlatform() == dai.Platform.RVC2:
    model_name = "luxonis/yolov8-instance-segmentation-nano:coco-512x288"
    setRunOnHost = True
    fps = 10

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    camera_node = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    det_nn = pipeline.create(dai.node.DetectionNetwork).build(camera_node, dai.NNModelDescription(model_name), fps=fps)
    det_nn.detectionParser.setRunOnHost(setRunOnHost)

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
    det_nn.passthrough.link(align.inputAlignTo)

    spatial_calculator = pipeline.create(dai.node.SpatialLocationCalculator)
    spatial_calculator.initialConfig.setUseSegmentation(True)
    align.outputAligned.link(spatial_calculator.inputDepth)
    det_nn.out.link(spatial_calculator.inputDetections)

    cam_queue = det_nn.passthrough.createOutputQueue()
    spatial_output_queue = spatial_calculator.outputDetections.createOutputQueue()
    depth_queue = spatial_calculator.passthroughDepth.createOutputQueue()

    input_config_queue = spatial_calculator.inputConfig.createInputQueue()
    calculatorConfig = spatial_calculator.initialConfig # copy initial settings

    pipeline.start()
    print("Pipeline created.")
    while pipeline.isRunning():
        in_spatial_det = spatial_output_queue.get()
        rgb_frame = cam_queue.get()
        depth_frame = depth_queue.get()

        assert isinstance(in_spatial_det, dai.SpatialImgDetections)
        assert isinstance(rgb_frame, dai.ImgFrame)
        assert isinstance(depth_frame, dai.ImgFrame)

        depthImg = depth_frame.getCvFrame()
        colorizedDepth = cv2.applyColorMap(cv2.convertScaleAbs(depthImg, alpha=0.03), cv2.COLORMAP_JET)
        image = rgb_frame.getCvFrame()

        segmentation_mask = cv2.Mat(np.zeros((in_spatial_det.getSegmentationMaskHeight(), in_spatial_det.getSegmentationMaskWidth()), dtype=np.uint8))
        segmentation_mask = in_spatial_det.getCvSegmentationMask()

        if segmentation_mask is not None and calculatorConfig.getUseSegmentation():
            scaled_mask = segmentation_mask.copy()
            scaled_mask[segmentation_mask != 255] = segmentation_mask[segmentation_mask != 255] * 25 # scale for better visualization
            colored_mask = cv2.applyColorMap(scaled_mask, cv2.COLORMAP_JET)
            colored_mask[segmentation_mask == 255] = image[segmentation_mask == 255]
            image = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)

        for i, det in enumerate(in_spatial_det.detections):
            outer_points = det.getBoundingBox()
            if outer_points is not None:
                outer_points = outer_points.denormalize(image.shape[1], image.shape[0])
                outer_points = outer_points.getPoints()
                outer_points = [[int(p.x), int(p.y)] for p in outer_points]
                outer_points = np.array(outer_points, dtype=np.int32)
                cv2.polylines(image, [outer_points], isClosed=True, color=(0, 255, 0), thickness=2)

                # depth
                depth_coordinate = det.spatialCoordinates
                depth = depth_coordinate.z
                text = f"X: {int(depth_coordinate.x / 10 )} cm, Y: {int(depth_coordinate.y / 10)} cm, Z: {int(depth / 10)} cm"
                cv2.putText(image, text, outer_points[0], cv2.FONT_HERSHEY_PLAIN, 1, (232,36,87), 2)

        top_panel = np.ones((100, image.shape[1], 3), dtype=np.uint8) * 255
        cv2.putText(top_panel, "Press 's' to toggle setUseSegmentation", (10, 30), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)
        cv2.putText(top_panel, f"Current setUseSegmentation: {calculatorConfig.getUseSegmentation()}", (10, 60), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 0, 0), 1)
        concatenated_frame = cv2.vconcat([top_panel, image])

        cv2.imshow("Depth", colorizedDepth)
        cv2.imshow("Spatial detections", concatenated_frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        elif key == ord('s'):
            calculatorConfig.setUseSegmentation(not calculatorConfig.getUseSegmentation())
            input_config_queue.send(calculatorConfig)
