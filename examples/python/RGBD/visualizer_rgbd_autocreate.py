import depthai as dai
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )
    # The Depth node manages its own stereo cameras and backend internally, and
    # RGBD aligns its depth to the color camera internally.
    color = p.create(dai.node.Camera).build()
    depth = p.create(dai.node.Depth)
    rgbd = p.create(dai.node.RGBD).build(color, depth)
    remoteConnector.addTopic("pcl", rgbd.pcl, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
