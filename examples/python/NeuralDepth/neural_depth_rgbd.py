import time
import depthai as dai

from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
args = parser.parse_args()

FPS = 10
with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.NeuralDepth)
    rgbd = p.create(dai.node.RGBD).build()
    align = None

    color.build(sensorFps=FPS)
    left.build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    right.build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

    # Linking
    stereo.build(left.requestFullResolutionOutput(), right.requestFullResolutionOutput(), dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)
    out = color.requestOutput((1280, 800), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
    align = p.create(dai.node.ImageAlign)
    stereo.depth.link(align.input)
    out.link(align.inputAlignTo)
    align.outputAligned.link(rgbd.inDepth)
    out.link(rgbd.inColor)
    remoteConnector.addTopic("pcl", rgbd.pcl, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
