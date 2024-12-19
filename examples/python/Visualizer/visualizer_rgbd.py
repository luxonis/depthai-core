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
    fps = 30.0
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    color.build()
    left.setCamera("left")
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setFps(fps)
    right.setCamera("right")
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setFps(fps)
    out = color.requestOutput((1280, 720))


    out.link(stereo.inputAlignTo)
    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setSubpixel(True)

    # Linking
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(rgbd.inDepth)
    out.link(rgbd.inColor)
    remoteConnector.addTopic("pcl", rgbd.pcl, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
