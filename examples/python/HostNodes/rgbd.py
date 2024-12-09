import time
import depthai as dai
import sys

from pathlib import Path
installExamplesStr = Path(__file__).absolute().parents[2] / 'install_requirements.py --install_rerun'
try:
    import rerun as rr
except ImportError:
    sys.exit("Critical dependency missing: Rerun. Please install it using the command: '{} {}' and then rerun the script.".format(sys.executable, installExamplesStr))

import cv2

class RerunNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()


    def run(self):
        rr.init("", spawn=True)
        rr.log("world", rr.ViewCoordinates.FLU)
        rr.log("world/ground", rr.Boxes3D(half_sizes=[3.0, 3.0, 0.00001])) 
        while self.isRunning():
            pclObstData = self.inputPCL.tryGet()
            if pclObstData is not None:
                print("runnnn")
                points, colors = pclObstData.getPointsRGB()
                rr.log("world/pcl", rr.Points3D(points, colors=colors, radii=[0.01]))

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    color = p.create(dai.node.ColorCamera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()

    rerunViewer = RerunNode()
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
    color.setFps(fps)

    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(left.getResolutionWidth(), left.getResolutionHeight())

    stereo.setExtendedDisparity(False)
    stereo.setLeftRightCheck(True)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.initialConfig.setLeftRightCheckThreshold(10)
    stereo.setSubpixel(True)


    # Linking

    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(rgbd.inDepth)
    color.preview.link(rgbd.inColor)

    rgbd.pcl.link(rerunViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
