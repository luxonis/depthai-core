import cv2
import depthai as dai


def printTransformation(label, transformation: dai.ImgTransformation):
    print(f"{label} transformation:")
    
    extrinsics: dai.Extrinsics = transformation.getExtrinsics()
    rotation = extrinsics.rotationMatrix
    translation = extrinsics.getTranslationVector()
    unit = extrinsics.lengthUnit
    
    extrinsics_matrix = [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0, 0, 0, 1]
    ]
    
    
    print("Extrinsics matrix:")
    for row in extrinsics_matrix:
        print("    ",row)
    print(f"Units: {unit}")
    print("toCameraSocket: ", extrinsics.toCameraSocket)
    print("\n")
    
    transformationMatrix = transformation.getTransformationMatrix()
    print(f"Transformation matrix")
    for row in transformationMatrix:
        print("    ", row)
    print("\n")
    
    sourceIntrinsics = transformation.getSourceIntrinsicMatrix()
    print(f"Source intrinsics matrix")
    for row in sourceIntrinsics:
        print("    ", row)
    print("\n")
    
    transformationMatrix = transformation.getTransformationMatrix()
    
    intrinsics = transformation.getIntrinsicMatrix()
    print(f"Intrinsics matrix = sourceIntrinsicMatrix @ TransformationMatrix)")
    for row in intrinsics:
        print("    ", row)
    print("\n")
    
    print("Source matrix inverse:")
    sourceIntrinsicsInv = transformation.getSourceIntrinsicMatrixInv()
    for row in sourceIntrinsicsInv:
        print("    ", row)
    print("\n")
    
    print("intrinsics inverse:")
    intrinsicsInv = transformation.getIntrinsicMatrixInv()
    for row in intrinsicsInv:
        print("    ", row)
    print("\n")
    
    print("Transformation matrix inverse:")
    transformationMatrixInv = transformation.getTransformationMatrixInv()
    for row in transformationMatrixInv:
        print("    ", row)
    print("\n")
    
    print(f"Distortion coefficients: {transformation.getDistortionCoefficients()}")
    
    
    source_width, source_height = transformation.getSourceSize()
    width, height = transformation.getSize()
    print(f"Source size: {source_width}x{source_height}")
    print(f"Size: {width}x{height}")
    
    
    print("==================================================================================================================\n")



fps = 15

device = dai.Device()
with dai.Pipeline(device) as pipeline:
    camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    
    camera_c_output = camera_c.requestOutput(
        (1280, 800), fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
    )
    camera_b_output = camera_b.requestOutput(
        (1280, 800), fps=fps, enableUndistortion=False, resizeMode=dai.ImgResizeMode.CROP
    )
    
    camera_a_output = camera_a.requestOutput(
        (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
    )
    
    stereo = pipeline.create(dai.node.StereoDepth).build(camera_b_output, camera_c_output, presetMode=dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    camera_a_output.link(stereo.inputAlignTo)
    
    
    cam_c_q = camera_c_output.createOutputQueue()
    cam_b_q = camera_b_output.createOutputQueue()
    cam_a_q = camera_a_output.createOutputQueue()
    depth_q = stereo.depth.createOutputQueue()
    
    pipeline.start()
    
    while pipeline.isRunning():
        cam_c_frame = cam_c_q.get()
        cam_b_frame = cam_b_q.get()
        cam_a_frame = cam_a_q.get()
        depth_frame = depth_q.get()
        
        
        assert isinstance(cam_c_frame, dai.ImgFrame)
        assert isinstance(cam_b_frame, dai.ImgFrame)
        assert isinstance(cam_a_frame, dai.ImgFrame)
        assert isinstance(depth_frame, dai.ImgFrame)
        
        cam_a_trans = cam_a_frame.getTransformation()
        cam_b_trans = cam_b_frame.getTransformation()
        cam_c_trans = cam_c_frame.getTransformation()
        depth_trans = depth_frame.getTransformation()
        
        assert cam_a_trans is not None
        assert cam_b_trans is not None
        assert cam_c_trans is not None
        assert depth_trans is not None
        
        printTransformation("CAM_A", cam_a_trans)
        printTransformation("CAM_B", cam_b_trans)
        printTransformation("CAM_C", cam_c_trans)
        printTransformation("DEPTH", depth_trans)
        
        
        cv2.imshow("CAM_A", cam_a_frame.getCvFrame())
        cv2.imshow("CAM_B", cam_b_frame.getCvFrame())
        cv2.imshow("CAM_C", cam_c_frame.getCvFrame())
        print("Depth frame size: ", depth_frame.getWidth(), "x", depth_frame.getHeight())
        
        cv2.waitKey(1)