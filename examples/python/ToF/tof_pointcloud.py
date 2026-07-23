#!/usr/bin/env python3
"""ToF colorized point cloud streamed to the DepthAI Visualizer.

Open http://localhost:8082 in a browser after starting this script.
"""

import depthai as dai

FPS = 10.0
SIZE = (640, 400)

with dai.Pipeline() as p:
    device = p.getDefaultDevice()

    if device.getPlatform() == dai.Platform.RVC2:
        RGB_SOCKET = dai.CameraBoardSocket.CAM_C
    else:
        RGB_SOCKET = dai.CameraBoardSocket.CAM_A

    color = p.create(dai.node.Camera).build(RGB_SOCKET, sensorFps=FPS)
    colorOut = color.requestOutput(SIZE, fps=FPS, type=dai.ImgFrame.Type.RGB888i, enableUndistortion =True)

    tof = p.create(dai.node.ToF)
    tof.build(
        boardSocket=dai.CameraBoardSocket.AUTO,
        presetMode=dai.ImageFiltersPresetMode.TOF_HIGH_RANGE,
        fps=FPS
    )

    align = p.create(dai.node.ImageAlign)
    align.setRunOnHost(True)
    tof.depth.link(align.input)
    colorOut.link(align.inputAlignTo)

    pc = p.create(dai.node.PointCloud)
    pc.initialConfig.setLengthUnit(dai.LengthUnit.MILLIMETER)
    pc.setTargetCoordinateSystem(RGB_SOCKET)
    align.outputAligned.link(pc.inputDepth)
    colorOut.link(pc.inputColor)

    remote = dai.RemoteConnection()
    remote.addTopic("pcl", pc.outputPointCloud, "3d")

    p.start()
    remote.registerPipeline(p)
    print("Visualizer running at http://localhost:8082")

    while p.isRunning():
        key = remote.waitKey(1)
        if key == ord("q"):
            break
