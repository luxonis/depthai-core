import depthai as dai
import sys
import numpy as np
import cv2

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        "Critical dependency missing: Open3D. Please install it using the command: '{} -m pip install open3d' and then rerun the script.".format(
            sys.executable
        )
    )

device = dai.Device()
fps = 30
frameType = dai.ImgFrame.Type.BGR888i
modelName = "luxonis/yolov8-large-pose-estimation:coco-640x352"
if device.getPlatform() == dai.Platform.RVC2:
    modelName = "luxonis/yolov8-nano-pose-estimation:coco-512x288"
    fps = 10
    frameType = dai.ImgFrame.Type.BGR888p

nnArchive = dai.NNArchive(dai.getModelFromZoo(dai.NNModelDescription(modelName,  device.getPlatformAsString())))
assert nnArchive is not None
w = nnArchive.getInputWidth()
h = nnArchive.getInputHeight()
assert w is not None
assert h is not None

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")

    cameraNode = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camOut =  cameraNode.requestOutput((w, h), frameType, fps= fps, enableUndistortion=True)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    stereo = pipeline.create(dai.node.StereoDepth)

    monoLeftOut = monoLeft.requestOutput((w, h))
    monoRightOut = monoRight.requestOutput((w, h))
    monoLeftOut.link(stereo.left)
    monoRightOut.link(stereo.right)
    stereo.setRectification(True)
    stereo.setExtendedDisparity(True)
    stereo.setLeftRightCheck(True)

    detNN = pipeline.create(dai.node.DetectionNetwork).build(camOut, nnArchive)
    align = pipeline.create(dai.node.ImageAlign)
    stereo.depth.link(align.input)
    detNN.passthrough.link(align.inputAlignTo)

    spatialCalculator = pipeline.create(dai.node.SpatialLocationCalculator)
    spatialCalculator.initialConfig.setCalculateSpatialKeypoints(True)
    align.outputAligned.link(spatialCalculator.inputDepth)
    detNN.out.link(spatialCalculator.inputDetections)

    camQueue = detNN.passthrough.createOutputQueue()
    spatialOutputQueue = spatialCalculator.outputDetections.createOutputQueue()
    depthQueue = spatialCalculator.passthroughDepth.createOutputQueue()
    pipeline.start()
    print("Pipeline created.")

    # Prepare Open3D visualization for spatial keypoints (converted to centimeters)
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Spatial Keypoints 3D", width=1280, height=720)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([200, 200, 200])
    opt.point_size = 15.0
    opt.line_width = 5.0
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[-1,-1,0])
    vis.add_geometry(axes)

    pc = o3d.geometry.PointCloud()
    lineSet = o3d.geometry.LineSet()

    vis.add_geometry(pc)
    vis.add_geometry(lineSet)
    vis.poll_events()
    vis.update_renderer()

    while pipeline.isRunning():
        spatialPointsCm = []
        lines = []
        pointCountOffset = 0
        spatialData = spatialOutputQueue.get()
        passthrough = camQueue.get()
        depthFrame = depthQueue.get()

        assert isinstance(spatialData, dai.SpatialImgDetections)
        assert isinstance(passthrough, dai.ImgFrame)
        assert isinstance(depthFrame, dai.ImgFrame)

        depthImg = depthFrame.getCvFrame()
        colorizedDepth = cv2.applyColorMap(cv2.convertScaleAbs(depthImg, alpha=0.03), cv2.COLORMAP_JET)
        image = passthrough.getCvFrame()

        filterKeypoints = [0, 3, 4, 7, 8, 13, 14, 15, 16] # filter out nose, ears, elbows, knees, ankles
        connectedKeypoints = [[0, 1], [2, 3], [2, 4], [3, 5], [2, 6], [3, 7]] # indices of keypoints that are connected with lines.

        for (i, det) in enumerate(spatialData.detections):
            bbox = det.getBoundingBox().denormalize(image.shape[1], image.shape[0])
            outerPoints = bbox.getPoints()

            outerPoints = [[int(p.x), int(p.y)] for p in outerPoints]
            outerPoints = np.array(outerPoints, dtype=np.int32)
            image = cv2.polylines(image, [outerPoints], isClosed=True, color=(0, 255, 0), thickness=2)
            depthCoordinate = det.spatialCoordinates
            depth = depthCoordinate.z
            text = f"X: {int(depthCoordinate.x / 10 )} cm, Y: {int(depthCoordinate.y / 10)} cm, Z: {int(depth / 10)} cm"
            cv2.putText(image, text, outerPoints[0], cv2.FONT_HERSHEY_PLAIN, 1, (232,36,87), 2)
            keypoints = det.getKeypoints()
            detIndices = []
            for (j, kp) in enumerate(keypoints):
                x = kp.imageCoordinates.x
                y = kp.imageCoordinates.y
                if (j in filterKeypoints):
                    continue

                image = cv2.circle(image, (int(x * image.shape[1]), int(y * image.shape[0])), 10, (255, 0, 0), -1)
                cv2.circle(colorizedDepth, (int(x * depthImg.shape[1]), int(y * depthImg.shape[0])), 10, (0, 0, 255), -1)
                cv2.putText(image,f"Z: {int(kp.spatialCoordinates.z / 10)} cm", (int(x * image.shape[1]), int(y * image.shape[0]) - 15), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255), 2)
                spatialPointsCm.append([-kp.spatialCoordinates.x / 1000.0,
                                        -kp.spatialCoordinates.y / 1000.0,
                                        kp.spatialCoordinates.z / 1000.0])
                detIndices.append(pointCountOffset)
                pointCountOffset += 1

            if (len(keypoints) > 0):
                for kp_indices in connectedKeypoints:
                    try:
                        idx1 = detIndices[kp_indices[0]]
                        idx2 = detIndices[kp_indices[1]]
                        lines.append([idx1, idx2])
                    except IndexError:
                        continue

        # origin=[0,0,0]. X=Red, Y=Green, Z=Blue
        npPoints = np.array(spatialPointsCm, dtype=np.float32)
        lineSet.points = o3d.utility.Vector3dVector(npPoints)
        lineSet.lines = o3d.utility.Vector2iVector(np.array(lines))
        lineSet.colors = o3d.utility.Vector3dVector(np.tile([0,1 ,0], (len(lines),1)))

        pc.points = o3d.utility.Vector3dVector(npPoints)
        # Color points bright magenta in 3D view
        colors = np.tile([1.0, 0.0, 1.0], (len(npPoints), 1))
        pc.colors = o3d.utility.Vector3dVector(colors)
        vis.update_geometry(pc)
        vis.update_geometry(lineSet)
        vis.poll_events()
        vis.update_renderer()

        cv2.imshow("depth", colorizedDepth)
        cv2.imshow("det", image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            vis.destroy_window()
            break
