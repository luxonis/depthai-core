#include "depthai/depthai.hpp"
#include "rerun_node.hpp"

int main() {
    using namespace std;
    // ULogger::setType(ULogger::kTypeConsole);
    // ULogger::setLevel(ULogger::kDebug);
    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto odom = pipeline.create<dai::node::BasaltVIO>();
    auto slam = pipeline.create<dai::node::RTABMapSLAM>();
    std::map<std::string, std::string> params{};
    params.insert(std::make_pair<std::string, std::string>("RGBD/CreateOccupancyGrid", "true"));
    params.insert(std::make_pair<std::string, std::string>("Grid/3D", "true"));
    params.insert(std::make_pair<std::string, std::string>("Rtabmap/SaveWMState", "true"));
    slam->setParams(params);
    auto rerun = pipeline.create<RerunNode>();

    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    odom->setImuUpdateRate(200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(true);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig->setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // Linking
    left->requestOutput(std::make_pair(width, height))->link(stereo->left);
    right->requestOutput(std::make_pair(width, height))->link(stereo->right);
    stereo->syncedLeft.link(odom->left);
    stereo->syncedRight.link(odom->right);
    stereo->depth.link(slam->depth);
    stereo->rectifiedLeft.link(slam->rect);
    imu->out.link(odom->imu);

    odom->transform.link(slam->odom);
    slam->transform.link(rerun->inputTrans);
    slam->passthroughRect.link(rerun->inputImg);
    slam->occupancyGridMap.link(rerun->inputMap);
    slam->obstaclePCL.link(rerun->inputObstaclePCL);
    slam->groundPCL.link(rerun->inputGroundPCL);
    pipeline.start();
    pipeline.wait();
}
