import time
import depthai as dai

from argparse import ArgumentParser

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
    fps = 10

    if args.depthSource == "stereo":
        color = p.create(dai.node.Camera).build(sensorFps=fps)
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=fps)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        depthSource = p.create(dai.node.StereoDepth)
        depthSource.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        depthSource.setRectifyEdgeFillColor(0)
        depthSource.enableDistortionCorrection(True)
        left.requestOutput(size, fps=fps).link(depthSource.left)
        right.requestOutput(size, fps=fps).link(depthSource.right)
    elif args.depthSource == "neural":
        color = p.create(dai.node.Camera).build(sensorFps=fps)
        left = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        depthSource = p.create(dai.node.NeuralDepth).build(left.requestOutput(size), right.requestOutput(size), dai.DeviceModelZoo.NEURAL_DEPTH_LARGE)
    elif args.depthSource == "tof":
        color = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=fps)
        socket, preset_mode = dai.CameraBoardSocket.AUTO, dai.ImageFiltersPresetMode.TOF_MID_RANGE
        depthSource = p.create(dai.node.ToF).build(socket, preset_mode)
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
