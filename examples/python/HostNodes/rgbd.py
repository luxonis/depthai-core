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
                points, colors = pclObstData.getPointsRGB()
                rr.log("world/pcl", rr.Points3D(points, colors=colors, radii=[0.01]))

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    color.build()
    rerunViewer = RerunNode()
    left.setCamera("left")
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    left.setFps(fps)
    right.setCamera("right")
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    right.setFps(fps)
    out = color.requestOutput((1280,720))

    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(left.getResolutionWidth(), left.getResolutionHeight())

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

    rgbd.pcl.link(rerunViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)