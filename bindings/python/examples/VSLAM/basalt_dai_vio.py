# class RerunStreamer : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunStreamer> {
#    public:
#     constexpr static const char* NAME = "RerunStreamer";

#    public:
#     void build() {}

#     Input inputTrans{*this, {.name = "inTrans", .types = {{dai::DatatypeEnum::TransformData, true}}}};
#     Input inputImg{*this, {.name = "inImg", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
#     Input inputObstaclePCL{*this, {.name = "inObstaclePCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
#     Input inputGroundPCL{*this, {.name = "inGroundPCL", .types = {{dai::DatatypeEnum::PointCloudData, true}}}};
#     Input inputMap{*this, {.name = "inMap", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};

#     void run() override {
#         const auto rec = rerun::RecordingStream("rerun");
#         rec.spawn().exit_on_failure();
#         rec.log_timeless("world", rerun::ViewCoordinates::FLU);
#         rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));
#         while(isRunning()) {
#             std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
#             auto imgFrame = inputImg.get<dai::ImgFrame>();
#             auto pclObstData = inputObstaclePCL.tryGet<dai::PointCloudData>();
#             auto pclGrndData = inputGroundPCL.tryGet<dai::PointCloudData>();
#             auto mapData = inputMap.tryGet<dai::ImgFrame>();
#             if(transData != nullptr) {
#                 auto trans = transData->getTranslation();
#                 auto quat = transData->getQuaternion();

#                 auto position = rerun::Vec3D(trans.x, trans.y, trans.z);

#                 rec.log("world/camera", rerun::Transform3D(position, rerun::datatypes::Quaternion::from_xyzw(quat.qx, quat.qy, quat.qz, quat.qw)));
#                 positions.push_back(position);
#                 rerun::LineStrip3D lineStrip(positions);
#                 rec.log("world/trajectory", rerun::LineStrips3D(lineStrip));
#                 rec.log("world/camera/image",
#                         rerun::Pinhole::from_focal_length_and_resolution({398.554f, 398.554f}, {640.0f, 400.0f})
#                             .with_camera_xyz(rerun::components::ViewCoordinates::FLU));
#                 rec.log("world/camera/image/rgb",
#                         rerun::Image(tensorShape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
# #ifdef DEPTHAI_HAVE_PCL_SUPPORT
#                 if(pclObstData != nullptr) {
#                     std::vector<rerun::Position3D> points;
#                     auto pclData = pclObstData->getPclData()->points;
#                     for(auto& point : pclData) {
#                         points.push_back(rerun::Position3D(point.x, point.y, point.z));
#                     }
#                     rec.log("world/obstacle_pcl", rerun::Points3D(points).with_radii({0.01f}));
#                 }
#                 if(pclGrndData != nullptr) {
#                     std::vector<rerun::Position3D> points;
#                     auto pclData = pclGrndData->getPclData()->points;
#                     for(auto& point : pclData) {
#                         points.push_back(rerun::Position3D(point.x, point.y, point.z));
#                     }
#                     rec.log("world/ground_pcl", rerun::Points3D(points).with_colors(rerun::Color{0, 255, 0}).with_radii({0.01f}));
#                 }
# #endif
#                 if(mapData != nullptr) {
#                     rec.log("map", rerun::Image(tensorShape(mapData->getCvFrame()), reinterpret_cast<const uint8_t*>(mapData->getCvFrame().data)));
#                 }
#             }
#         }
#     }
#     std::vector<rerun::Vec3D> positions;
# };

import time
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
        # rr.log("world/ground", rr.Boxes3D.half_sizes([[3.0, 3.0, 0.00001]])) -doesn't work for some reason 
        while self.isRunning():
            transData = self.inputTrans.get()
            imgFrame = self.inputImg.get()
            pclObstData = self.inputObstaclePCL.tryGet()
            pclGrndData = self.inputGroundPCL.tryGet()
            mapData = self.inputGrid.tryGet()
            if transData is not None:
                trans = transData.getTranslation()
                quat = transData.getQuaternion()
                position = rr.Vec3D(trans.x, trans.y, trans.z)
                rr.log("world/camera", rr.Transform3D(position, rr.Quaternion.from_xyzw(quat.qx, quat.qy, quat.qz, quat.qw)))
                self.positions.append(position)
                lineStrip = rr.LineStrip3D(self.positions)
                rr.log("world/trajectory", rr.LineStrips3D(lineStrip))
                rr.log("world/camera/image", rr.Pinhole.from_focal_length_and_resolution([398.554, 398.554], [640.0, 400.0]).with_camera_xyz(rr.ViewCoordinates.FLU))
                rr.log("world/camera/image/rgb", rr.Image(imgFrame.getCvFrame()))
                if pclObstData is not None:
                    points = []
                    pclData = pclObstData.getPclData().points
                    for point in pclData:
                        points.append(rr.Position3D(point.x, point.y, point.z))
                    rr.log("world/obstacle_pcl", rr.Points3D(points).with_radii([0.01]))
                if pclGrndData is not None:
                    points = []
                    pclData = pclGrndData.getPclData().points
                    for point in pclData:
                        points.append(rr.Position3D(point.x, point.y, point.z))
                    rr.log("world/ground_pcl", rr.Points3D(points).with_colors(rr.Color(0, 255, 0)).with_radii([0.01]))
                if mapData is not None:
                    rr.log("map", rr.Image(mapData.getCvFrame()))



# Create pipeline

with dai.Pipeline() as p:
    fps = 60
    width = 640
    height = 400
    # Define sources and outputs
    left = p.create(dai.node.MonoCamera)
    right = p.create(dai.node.MonoCamera)
    imu = p.create(dai.node.IMU)
    odom = p.create(dai.node.BasaltVIO)
    odom = odom.build()

    rerunViewer = RerunNode()
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    left.setCamera("left")
    left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    left.setFps(fps)
    right.setCamera("right")
    right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    right.setFps(fps)

    # Linking

    left.out.link(odom.left)
    right.out.link(odom.right)
    imu.out.link(odom.imu)
    odom.passthrough.link(rerunViewer.inputImg)
    odom.transform.link(rerunViewer.inputTrans)
    # odom.passthrough
    p.start()
    while True:
        time.sleep(1)
        print("Pipeline is running...")