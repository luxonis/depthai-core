import time
import depthai as dai

from argparse import ArgumentParser

NEURAL_FPS = 8
STEREO_DEFAULT_FPS = 30
TOF_DEFAULT_FPS = 30

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
parser.add_argument("--depthSource", type=str, default="stereo", choices=["stereo", "neural", "tof"])
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )

    size = (640, 400)
    if args.depthSource == "neural":
        fps = NEURAL_FPS
    elif args.depthSource == "tof":
        fps = TOF_DEFAULT_FPS
    else:
        fps = STEREO_DEFAULT_FPS

    if args.depthSource == "stereo":
        color = p.create(dai.node.Camera).build(sensorFps=fps)
        # The Depth node manages its own stereo cameras and backend internally, so
        # no explicit left/right cameras are needed. RGBD aligns its depth to the
        # color camera internally.
        depthSource = p.create(dai.node.Depth).build(fps=fps)
    elif args.depthSource == "neural":
        color = p.create(dai.node.Camera).build(sensorFps=fps)
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        depthSource = p.create(dai.node.NeuralDepth).build(left.requestOutput(size), right.requestOutput(size), dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)
    elif args.depthSource == "tof":
        color = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        socket, profile = dai.CameraBoardSocket.AUTO, dai.ToFConfig.Profile.MID_RANGE
        depthSource = p.create(dai.node.ToF).build(socket, profile)
    else:
        raise ValueError(f"Invalid depth source: {args.depthSource}")

    rgbd = p.create(dai.node.RGBD).build(color, depthSource, size, fps)

    remoteConnector.addTopic("pcl", rgbd.pcl, "common")
    p.start()
    remoteConnector.registerPipeline(p)

    print("Pipeline started with depth source: ", args.depthSource)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
