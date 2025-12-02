import os
import sys

sys.path.insert(0, '/home/tomas/code/depthai-device-kb/external/depthai-core/build/bindings/python/')
os.environ["DEPTHAI_LEVEL"] = "info"
os.environ["DEPTHAI_DEVICE_RVC4_FWP"] = "/home/tomas/code/depthai-device-kb/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4-fwp.tar.xz"

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
        while self.isRunning():
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


# Target device IP
TARGET_IP = "10.11.1.218"
deviceInfo = dai.DeviceInfo(TARGET_IP)
device = dai.Device(deviceInfo)

# Read current calibration
calibration_handler = device.readCalibration()
eepromData = calibration_handler.getEepromData()

# Prepare extrinsics
rotation_x_90 = np.array([
    [1, 0, 0],
    [0, 0, -1],
    [0, 1, 0]
]).tolist()
rotation_none = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
]).tolist()

# Set toCameraSocket first
eepromData.housingExtrinsics.toCameraSocket = dai.CameraBoardSocket.CAM_C

# Set rotation and translation
eepromData.housingExtrinsics.rotationMatrix = rotation_none
eepromData.housingExtrinsics.translation.x = 10000
eepromData.housingExtrinsics.translation.y = 0
eepromData.housingExtrinsics.translation.z = 0

# Set specTranslation
eepromData.housingExtrinsics.specTranslation.x = 10000
eepromData.housingExtrinsics.specTranslation.y = 0
eepromData.housingExtrinsics.specTranslation.z = 0

# Apply new calibration
calibration_handler = dai.CalibrationHandler(eepromData)
device.flashCalibration(calibration_handler)
device.setCalibration(calibration_handler)

# Verify
src_camera = dai.CameraBoardSocket.CAM_A
T_src_to_housing = calibration_handler.getHousingCalibration(
    srcCamera=src_camera,
    housingCS=dai.HousingCoordinateSystem.VESA_RIGHT,
    useSpecTranslation=True
)
print("Source Camera to Housing Calibration 4x4 Matrix:")
for row in T_src_to_housing:
    print(row)

print(calibration_handler.getCameraExtrinsics(dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B))

with dai.Pipeline(device) as p:
    p.setCalibrationData(calibration_handler)
    fps = 30
    # Define sources and outputs
    left = p.create(dai.node.Camera)
    right = p.create(dai.node.Camera)
    color = p.create(dai.node.Camera)
    stereo = p.create(dai.node.StereoDepth)
    rgbd = p.create(dai.node.RGBD).build()
    align = None
    color.build()
    o3dViewer = O3DNode()
    left.build(dai.CameraBoardSocket.CAM_B)
    right.build(dai.CameraBoardSocket.CAM_C)
    out = None

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
    stereo.setRectifyEdgeFillColor(0)
    stereo.enableDistortionCorrection(True)

    # Linking
    left.requestOutput((640, 400)).link(stereo.left)
    right.requestOutput((640, 400)).link(stereo.right)
    platform = p.getDefaultDevice().getPlatform()
    if platform == dai.Platform.RVC4:
        out = color.requestOutput((640, 400), dai.ImgFrame.Type.RGB888i)
        align = p.create(dai.node.ImageAlign)
        stereo.depth.link(align.input)
        out.link(align.inputAlignTo)
        align.outputAligned.link(rgbd.inDepth)
    else:
        out = color.requestOutput(
            (640, 400), dai.ImgFrame.Type.RGB888i, dai.ImgResizeMode.CROP, 30, True
        )
        stereo.depth.link(rgbd.inDepth)
        out.link(stereo.inputAlignTo)
    out.link(rgbd.inColor)

    rgbd.pcl.link(o3dViewer.inputPCL)

    p.start()
    while p.isRunning():
        time.sleep(1)
