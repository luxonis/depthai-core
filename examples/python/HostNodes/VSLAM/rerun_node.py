import depthai as dai
import rerun as rr

class RerunNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputTrans = dai.Node.Input(self)
        self.inputImg = dai.Node.Input(self)
        self.inputObstaclePCL = dai.Node.Input(self)
        self.inputGroundPCL = dai.Node.Input(self)
        self.inputGrid = dai.Node.Input(self)
        self.positions = []
    def run(self):
        rr.init("", spawn=True)
        rr.log("world", rr.ViewCoordinates.FLU)
        rr.log("world/ground", rr.Boxes3D(half_sizes=[3.0, 3.0, 0.00001])) 
        while self.isRunning():
            transData = self.inputTrans.get()
            imgFrame = self.inputImg.get()
            pclObstData = self.inputObstaclePCL.tryGet()
            pclGrndData = self.inputGroundPCL.tryGet()
            mapData = self.inputGrid.tryGet()
            if transData is not None:
                trans = transData.getTranslation()
                quat = transData.getQuaternion()
                position = rr.datatypes.Vec3D([trans.x, trans.y, trans.z])
                rr.log("world/camera", rr.Transform3D(translation=position, rotation=rr.datatypes.Quaternion(xyzw=[quat.qx, quat.qy, quat.qz, quat.qw])))
                self.positions.append(position)
                lineStrip = rr.components.LineStrip3D(self.positions)
                rr.log("world/trajectory", rr.LineStrips3D(lineStrip))
                rr.log("world/camera/image", rr.Pinhole(resolution=[640.0, 400.0], focal_length=[398.554, 398.554], camera_xyz=rr.ViewCoordinates.FLU))
                rr.log("world/camera/image/rgb", rr.Image(imgFrame.getCvFrame()))
                if pclObstData is not None:
                    points = []
                    pclData = pclObstData.getPoints()
                    for point in pclData:
                        points.append(rr.datatypes.Vec3D([point[0], point[1], point[2]]))
                    rr.log("world/obstacle_pcl", rr.Points3D(points, radii=[0.01]))
                if pclGrndData is not None:
                    points = []
                    pclData = pclGrndData.getPoints()
                    for point in pclData:
                        points.append(rr.datatypes.Vec3D([point[0], point[1], point[2]]))
                    rr.log("world/ground_pcl", rr.Points3D(points, colors=rr.components.Color([0,255,0]), radii=[0.01]))
                if mapData is not None:
                    rr.log("map", rr.Image(mapData.getCvFrame()))