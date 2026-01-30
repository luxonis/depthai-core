import time
import depthai as dai
import sys

from pathlib import Path
installExamplesStr = Path(__file__).absolute().parents[1] / 'install_requirements.py --install_rerun'
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
        rr.log("world", rr.ViewCoordinates.RDF)
        rr.log("world/ground", rr.Boxes3D(half_sizes=[3.0, 3.0, 0.00001]))
        while self.mainLoop():
            try:
                inPointCloud = self.inputPCL.get()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if inPointCloud is not None:
                points, colors = inPointCloud.getPointsRGB()
                rr.log("world/pcl", rr.Points3D(points, colors=colors, radii=[0.01]))

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    align = None
    color.build()
    rerunViewer = RerunNode()
    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)
    out = None

    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.initialConfig.postProcessing.thresholdFilter.maxRange = 10000
    rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

    # Linking
    left.requestOutput((640, 400)).link(stereo.left)
    right.requestOutput((640, 400)).link(stereo.right)
    platform = p.getDefaultDevice().getPlatform()

    if platform == dai.Platform.RVC4:
        out = color.requestOutput((640,400), dai.ImgFrame.Type.RGB888i, enableUndistortion=True)
        align = p.create(dai.node.ImageAlign)
        stereo.depth.link(align.input)
        out.link(align.inputAlignTo)
        align.outputAligned.link(rgbd.inDepth)
    else:
        out = color.requestOutput((640,400), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True)
        stereo.depth.link(rgbd.inDepth)
        out.link(stereo.inputAlignTo)
    out.link(rgbd.inColor)

    rgbd.pcl.link(rerunViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
