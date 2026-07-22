import depthai as dai

from argparse import ArgumentParser

fps = 30

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )

    size = (640, 400)
    # Pick a COLOR-capable socket for the color camera so it does not take the
    # socket the Depth node needs (e.g. the ToF sensor on ToF-only devices, where
    # the default AUTO socket would otherwise grab the only ToF-capable sensor).
    colorSocket = dai.CameraBoardSocket.CAM_A
    for features in p.getDefaultDevice().getConnectedCameraFeatures():
        if dai.CameraSensorType.COLOR in features.supportedTypes:
            colorSocket = features.socket
            break
    color = p.create(dai.node.Camera).build(colorSocket, sensorFps=fps)
    # The Depth node manages its own stereo cameras and backend internally, so
    # no explicit left/right cameras are needed. RGBD aligns its depth to the
    # color camera internally.
    depth = p.create(dai.node.Depth).build(fps=fps)

    rgbd = p.create(dai.node.RGBD).build(color, depth, size, fps)

    remoteConnector.addTopic("pcl", rgbd.pcl, "common")
    p.start()
    remoteConnector.registerPipeline(p)

    print("Pipeline started")

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
