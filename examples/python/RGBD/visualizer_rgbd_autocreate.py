import depthai as dai
from argparse import ArgumentParser

# NOTE: Using autocreate takes over the cameras cannot be used in complex pipelines,
# where cameras would be used in other nodes as well yet.

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )
    colorSockets = p.getDefaultDevice().getConnectedCameras(dai.CameraSensorType.COLOR)
    colorSocket = colorSockets[0] if colorSockets else dai.CameraBoardSocket.CAM_A
    color = p.create(dai.node.Camera).build(colorSocket)
    depth = p.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, None, (640, 400))
    rgbd = p.create(dai.node.RGBD).build(color, depth)
    remoteConnector.addTopic("pcl", rgbd.pcl, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
