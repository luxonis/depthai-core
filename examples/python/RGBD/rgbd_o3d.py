
import time
import depthai as dai
import sys
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit("Critical dependency missing: Open3D. Please install it using the command: '{} -m pip install open3d' and then rerun the script.".format(sys.executable))


class O3DNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()


    def run(self):
        def key_callback(vis, action, mods):
            global isRunning
            if action == 0:
                isRunning = False
        pc = o3d.geometry.PointCloud()
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.register_key_action_callback(81, key_callback)
        pcd = o3d.geometry.PointCloud()
        coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1000, origin=[0,0,0])
        vis.add_geometry(coordinateFrame)
        first = True
        while self.isRunning():
            inPointCloud = self.inputPCL.tryGet()
            if inPointCloud is not None:
                points, colors = inPointCloud.getPointsRGB()
                pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                colors = (colors.reshape(-1, 3) / 255.0).astype(np.float64)
                pcd.colors = o3d.utility.Vector3dVector(colors)
                if first:
                    vis.add_geometry(pcd)
                    first = False
                else:
                    vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()
        vis.destroy_window()

# Create pipeline

with dai.Pipeline() as p:
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    color.build()
    o3dViewer = O3DNode()
    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)
    out = color.requestOutput((1280,720), dai.ImgFrame.Type.RGB888i)


    out.link(stereo.inputAlignTo)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)

    # Linking
    left.requestOutput((1280, 720)).link(stereo.left)
    right.requestOutput((1280, 720)).link(stereo.right)
    stereo.depth.link(rgbd.inDepth)
    out.link(rgbd.inColor)

    rgbd.pcl.link(o3dViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
