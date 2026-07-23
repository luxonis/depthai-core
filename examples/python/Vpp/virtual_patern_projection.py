import cv2 as cv
import depthai as dai

"""
  ┌──────┐       ┌───────────────┐ ----------------left------------------------------> ┌─────┐
  | Left | ----> |               |                                                     |     |
  └──────┘       |               | ----------------right-----------------------------> |     | -left--> ┌────────┐
                 | Rectification |                                                     | Vpp |          | Stereo | --depth->
  ┌───────┐      |               | --left_low_res---> ┌──────────────┐ --disparity---> |     |          |        |
  | Right | ---> |               |                    | NeuralStereo |                 |     | -right-> └────────┘
  └───────┘      └───────────────┘ --right_low_res--> └──────────────┘ --confidence--> └─────┘
"""


if __name__ == "__main__":
    fps = 20

    device = dai.Device()
    pipeline = dai.Pipeline(device)

    if not device.isNeuralDepthSupported():
        print("Exiting Vpp example: device doesn't support NeuralDepth.")
        exit()

    # Left Right cameras
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    # Rectification node
    rectification = pipeline.create(dai.node.Rectification)
    monoLeftOut.link(rectification.input1)
    monoRightOut.link(rectification.input2)

    # Neural Depth
    neuralDepth = pipeline.create(dai.node.NeuralDepth).build(
        monoLeftOut,
        monoRightOut,
        dai.DeviceModelZoo.NEURAL_DEPTH_NANO
    )

    # Vpp node
    vpp = pipeline.create(dai.node.Vpp).build(
        rectification.output1,
        rectification.output2,
        neuralDepth.disparity,
        neuralDepth.confidence
    )
    # set initial parameters
    vpp.initialConfig.blending = 0.5
    vpp.initialConfig.maxPatchSize = 2
    vpp.initialConfig.patchColoringType = dai.VppConfig.PatchColoringType.RANDOM
    vpp.initialConfig.uniformPatch = True
    vpp.initialConfig.maxFPS = fps
    injection_params = vpp.initialConfig.injectionParameters
    injection_params.textureThreshold = 3.0
    injection_params.useInjection = True

    # for runtime parameter control use vpp.inputConfig queue
    # config_queue = vpp.inputConfig.createInputQueue()
    # config_queue.send(vpp_config)

    # Stereo node
    stereo_node = pipeline.create(dai.node.StereoDepth)
    stereo_node.setRectification(False)
    vpp.leftOut.link(stereo_node.left)
    vpp.rightOut.link(stereo_node.right)

    # Create output queues we want to show
    vpp_output_left = vpp.leftOut.createOutputQueue()
    vpp_output_right = vpp.rightOut.createOutputQueue()
    depth_queue = stereo_node.depth.createOutputQueue()

    # Start the pipeline
    with pipeline:
        pipeline.start()
        print("Pipeline started successfully")
        while pipeline.isRunning():
            vpp_out_left = vpp_output_left.get()
            vpp_out_right = vpp_output_right.get()
            depth = depth_queue.get()

            cv.imshow("vpp_left", vpp_out_left.getCvFrame())
            cv.imshow("vpp_right", vpp_out_right.getCvFrame())
            cv.imshow("Depth", dai.colorizeDepthFrame(depth, 500, 12000, cv.COLORMAP_TURBO, False).getCvFrame())

            key = cv.waitKey(1)
            if key == ord('q'):
                quit()
