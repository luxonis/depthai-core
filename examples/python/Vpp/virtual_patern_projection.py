import numpy as np
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


def showDepth(depthFrame, windowName="Depth", minDistance=500, maxDistance=5000,
               colormap=cv.COLORMAP_TURBO, useLog=False):
    """
    Nicely visualize a depth map.

    Args:
        depthFrame (np.ndarray): Depth frame (in millimeters).
        window_name (str): OpenCV window name.
        minDistance (int): Minimum depth to display (in mm).
        maxDistance (int): Maximum depth to display (in mm).
        colormap (int): OpenCV colormap (e.g., cv.COLORMAP_JET, COLORMAP_TURBO, etc.).
        use_log (bool): Apply logarithmic scaling for better visual contrast.
    """
    # Convert to float for processing
    depthFrame = depthFrame.astype(np.float32)

    # Optionally apply log scaling
    if useLog:
        depthFrame = np.log(depthFrame + 1)

    # Clip to defined range (avoid far-out values)
    depthFrame = np.uint8(np.clip(depthFrame, minDistance, maxDistance) / maxDistance * 255)

    # Apply color map
    depthColor = cv.applyColorMap(depthFrame, colormap)

    # Show in a window
    cv.imshow(windowName, depthColor)


if __name__ == "__main__":
    fps = 20

    pipeline = dai.Pipeline()

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
            showDepth(
                depth.getCvFrame(),
                windowName="Depth",
                minDistance=500,
                maxDistance=5000,
                colormap=cv.COLORMAP_TURBO,
                useLog=False
            )

            key = cv.waitKey(1)
            if key == ord('q'):
                quit()
