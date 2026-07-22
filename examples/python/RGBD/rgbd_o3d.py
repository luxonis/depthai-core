import time
import depthai as dai
import sys
import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        "Critical dependency missing: Open3D. Please install it using the command: '{} -m pip install open3d' and then rerun the script.".format(
            sys.executable
        )
    )


class O3DNode(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.inputPCL = self.createInput()

    def run(self):
        def key_callback(vis, action, mods):
            global isRunning
            if action == 0:
                isRunning = False

        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()
        vis.register_key_action_callback(81, key_callback)
        pcd = o3d.geometry.PointCloud()
        coordinateFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1000, origin=[0, 0, 0]
        )
        vis.add_geometry(coordinateFrame)
        first = True
        while self.mainLoop():
            try:
                inPointCloud = self.inputPCL.tryGet()
            except dai.MessageQueue.QueueException:
                return # Pipeline closed
            if inPointCloud is not None:
                points, colors = inPointCloud.getPointsRGB()
                pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
                colors = (colors / 255.0).astype(np.float64)
                pcd.colors = o3d.utility.Vector3dVector(np.delete(colors, 3, 1))
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
    # explicit left/right cameras or StereoDepth node are needed.
    depth = p.create(dai.node.Depth)
    o3dViewer = p.create(O3DNode)

    # RGBD wires the color camera and aligns the Depth node's depth to it
    # internally (using the Depth node's own alignment), so no ImageAlign node is
    # needed.
    rgbd = p.create(dai.node.RGBD).build(color, depth, (640, 400), fps)

    rgbd.pcl.link(o3dViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
