import depthai as dai


device = dai.Device()
platform = device.getPlatform()

with dai.Pipeline(device) as pipeline:
    print("Creating pipeline...")
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    left_steam = left.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    right_steam = right.requestFullResolutionOutput(type=dai.ImgFrame.Type.NV12)

    stereo = pipeline.create(dai.node.StereoDepth).build(
        left=left_steam,
        right=right_steam
    )

    stereo.rectifiedLeft.createOutputQueue().addCallback(lambda msg: print(f"Received right"))
    
    manip_left = pipeline.create(dai.node.ImageManipV2)
    manip_left.setRunOnHost(True)
    # COMMENT OUT EITHER OF THE NEXT TWO LINES TO GET IT WORKING
    manip_left.initialConfig.setOutputSize(320,240)
    manip_left.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
    ############################################################
    stereo.rectifiedLeft.link(manip_left.inputImage)

    manip_left.out.createOutputQueue().addCallback(lambda msg: print("Received manip img"))

    print("Pipeline created.")
    pipeline.run()

