import time
import depthai as dai

from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8080)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    stereo = p.create(dai.node.NeuralDepth)
    rgbd = p.create(dai.node.RGBD).build()
    align = None

    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)
    out = None


    # Linking
    left.requestFullResolutionOutput().link(stereo.left)
    right.requestFullResolutionOutput().link(stereo.right)
    out = stereo.rectification.output1
    out.link(rgbd.inColor)
    stereo.depth.link(rgbd.inDepth)
    remoteConnector.addTopic("pcl", rgbd.pcl, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
