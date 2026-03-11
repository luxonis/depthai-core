
    device = dai.Device()
    print("Sensor names:", device.getCameraSensorNames())

    # Detailed feature info, including supported configs/resolutions
    for cam in device.getConnectedCameraFeatures():
        print(f"\nSocket: {cam.socket}")
        print(f"Sensor: {cam.sensorName}")
        print(f"Max/native: {cam.width}x{cam.height}")
        print(f"Supported types: {cam.supportedTypes}")

        # Each config is a supported sensor mode
        for cfg in cam.configs:
            print("  ", cfg)

    try:
        with dai.Pipeline(device) as pipeline:

            def signal_handler(sig, frame):
                print("Interrupted, stopping the pipeline")
                pipeline.stop()

            signal.signal(signal.SIGINT, signal_handler)

            sync_node = pipeline.create(dai.node.Sync)
            sync_node.setRunOnHost(False)
            sync_node.setSyncThreshold(timedelta(milliseconds=1000 / fps * 0.5))
            sync_node.setSyncAttempts(-1)

            demux = pipeline.create(dai.node.MessageDemux)
            sync_node.out.link(demux.input)

            camera_a = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            camera_b = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            camera_c = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
            cameras = {"CAM_A": camera_a, "CAM_B": camera_b, "CAM_C": camera_c}

            reference_camera = cameras[reference_camera_name]

            # if not rectified:
            camera_c_output = camera_c.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )
            camera_b_output = camera_b.requestOutput(
                (1280, 800), fps=fps, enableUndistortion=True, resizeMode=dai.ImgResizeMode.CROP
            )
            # else:
            #     camera_c_output = camera_c.requestOutput(
            #         target_camera_size,
            #         fps=fps,
            #         enableUndistortion=target_camera_undistortion,
            #         resizeMode=target_camera_resize_mode,
            #     )
            #     camera_b_output = camera_b.requestOutput(
            #         target_camera_size,
            #         fps=fps,
            #         enableUndistortion=target_camera_undistortion,
            #         resizeMode=target_camera_resize_mode,
            #     )

            reference_camera_output = reference_camera.requestOutput(
                ref_camera_size,
                fps=fps,
                enableUndistortion=ref_camera_undistortion,
                resizeMode=ref_camera_resize_mode,
            )

            stereo = pipeline.create(dai.node.StereoDepth).build(camera_b_output, camera_c_output, presetMode=dai.node.StereoDepth.PresetMode.FAST_DENSITY)
            # reference_camera_output.link(stereo.inputAlignTo)

            # reference_camera_output = stereo.rectifiedRight
            reference_camera_output.link(sync_node.inputs["ref_camera"])
            link_record_video_node(
                pipeline,
                demux.outputs["ref_camera"],
                f"{output_path}{reference_camera_name}_reference",
                fps=fps,
            )

            if device.getPlatform() == dai.Platform.RVC4:
                
                image_align = pipeline.create(dai.node.ImageAlign)
                stereo.depth.link(image_align.input)
                reference_camera_output.link(image_align.inputAlignTo)
                image_align.outputAligned.link(sync_node.inputs["depth_aligned"])
            else:
                stereo.depth.link(sync_node.inputs["depth_aligned"])

            link_record_metadata_node(pipeline, demux.outputs["depth_aligned"], f"{output_path}_ALIGNED_DEPTH")

            target_camera = cameras[target_camera_name]
            # if not rectified:
            target_camera_output = target_camera.requestOutput(
                target_camera_size,
                fps=fps,
                enableUndistortion=target_camera_undistortion,
                resizeMode=target_camera_resize_mode,
            )
            # else:
            #     if target_camera_name == "CAM_B":
            #         target_camera_output = stereo.rectifiedLeft
            #     else:
            #         target_camera_output = stereo.rectifiedRight

            # target_camera_output = camera_b_output
            target_camera_output.link(sync_node.inputs["target_camera"])
            
            # target_fuller_res = target_camera.requestOutput((1920, 1080), fps=fps, enableUndistortion=target_camera_undistortion, resizeMode=dai.ImgResizeMode.CROP)
            # imgManip = pipeline.create(dai.node.ImageManip)
            # imgManip.initialConfig.addFlipHorizontal()
            # imgManip.initialConfig.addFlipVertical()
            # imgManip.initialConfig.addRotateDeg(9)
            # imgManip.initialConfig.setOutputSize(1478, 600, dai.ImageManipConfig.ResizeMode.STRETCH)
            # imgManip.setMaxOutputFrameSize(1920*1080*3)
            # target_fuller_res.link(imgManip.inputImage)
            
            # imgManip.out.link(sync_node.inputs["target_camera"])

            rectified_suffix = "_R" if rectified else ""
            link_record_video_node(
                pipeline,
                demux.outputs["target_camera"],
                f"{output_path}_{target_camera_name}{rectified_suffix}_target",
                fps=fps,
            )

            sync_output_queue = sync_node.out.createOutputQueue(maxSize=1, blocking=True)
            print(f"Writing ChArUco points to: {ref_points_mcap_path}")

            pipeline.start()
            seq_num = 0

            while pipeline.isRunning():
                assert sync_output_queue is not None
                message_group = sync_output_queue.get()
                assert isinstance(message_group, dai.MessageGroup)