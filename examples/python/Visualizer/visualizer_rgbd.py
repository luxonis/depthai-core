import time
import depthai as dai

from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8080)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
    fps = 30
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    color = p.create(dai.node.ColorCamera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()

    left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setFps(fps)
    right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setFps(fps)
    color.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    color.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    color.setInterleaved(False)
    color.setIspScale(2, 3)

    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(left.getResolutionWidth(), left.getResolutionHeight())

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setSubpixel(True)


    remoteConnector.addTopic("pcl", rgbd.pcl, "common")
    remoteConnector.addTopic("depth", stereo.depth, "common")
    remoteConnector.addTopic("color", color.video, "common")
    # Linking

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(rgbd.inDepth)
    color.video.link(rgbd.inColor)


    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
