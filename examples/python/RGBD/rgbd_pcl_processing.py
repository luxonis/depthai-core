import depthai as dai
import numpy as np
from argparse import ArgumentParser


class CustomPCLProcessingNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()
        self.outputPCL = self.createOutput()
        self.thresholdDistance = 3000.0

    def run(self):
        while self.isRunning():
            try:
                inPointCloud = self.inputPCL.get()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if inPointCloud is not None:
                outPointCloud = dai.PointCloudData()
                points, colors = inPointCloud.getPointsRGB()

                mask = np.linalg.norm(points, axis=1) < self.thresholdDistance
                updatedPoints = points[mask]
                updatedColors = colors[mask]

                outPointCloud.setPointsRGB(updatedPoints, updatedColors)
                self.outputPCL.send(outPointCloud)

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8080)
args = parser.parse_args()

with dai.Pipeline() as p:
    remoteConnector = dai.RemoteConnection(
        webSocketPort=args.webSocketPort, httpPort=args.httpPort
    )
    rgbd = p.create(dai.node.RGBD).build(True, dai.node.StereoDepth.PresetMode.DEFAULT)
    customNode = p.create(CustomPCLProcessingNode)

    # Link rgbd.pcl to the input of CustomPCLProcessingNode
    rgbd.pcl.link(customNode.inputPCL)

    remoteConnector.addTopic("pcl", rgbd.pcl, "common")
    remoteConnector.addTopic("processed_pcl", customNode.outputPCL, "common")

    p.start()
    remoteConnector.registerPipeline(p)

    while p.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
