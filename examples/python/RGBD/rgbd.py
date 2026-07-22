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
    # Pick a COLOR-capable socket for the color camera so it does not take the
    # socket the Depth node needs (e.g. the ToF sensor on ToF-only devices, where
    # the default AUTO socket would otherwise grab the only ToF-capable sensor).
    colorSocket = dai.CameraBoardSocket.CAM_A
    for features in p.getDefaultDevice().getConnectedCameraFeatures():
        if dai.CameraSensorType.COLOR in features.supportedTypes:
            colorSocket = features.socket
            break
    color = p.create(dai.node.Camera).build(colorSocket)
    # The Depth node manages its own stereo cameras and backend internally, so no
    # explicit left/right cameras or StereoDepth node are needed. The (640, 400)
    # size keeps the depth resolution the same as the RGBD frame size instead of
    # computing depth at the full stereo sensor resolution.
    depth = p.create(dai.node.Depth).build(dai.node.Depth.Algorithm.AUTO, fps, (640, 400))
    rerunViewer = p.create(RerunNode)

    # RGBD wires the color camera and aligns the Depth node's depth to it
    # internally (using the Depth node's own alignment), so no ImageAlign node is
    # needed.
    rgbd = p.create(dai.node.RGBD).build(color, depth, (640, 400), fps)
    rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

    rgbd.pcl.link(rerunViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
