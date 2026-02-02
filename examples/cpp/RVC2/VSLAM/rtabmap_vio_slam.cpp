
#include "depthai/depthai.hpp"
#include "rerun_node.hpp"

int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 30;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto featureTracker = pipeline.create<dai::node::FeatureTracker>();
    auto odom = pipeline.create<dai::node::RTABMapVIO>();
    auto slam = pipeline.create<dai::node::RTABMapSLAM>();
    auto rerun = pipeline.create<RerunNode>();
    std::map<std::string, std::string> params{};
    params.insert(std::make_pair<std::string, std::string>("Odom/ResetCountDown", "30"));

    odom->setParams(params);

    params.insert(std::make_pair<std::string, std::string>("RGBD/CreateOccupancyGrid", "true"));
    params.insert(std::make_pair<std::string, std::string>("Grid/3D", "true"));
    slam->setParams(params);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig->setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig->setNumTargetFeatures(1000);
    featureTracker->initialConfig->setMotionEstimator(false);
    featureTracker->initialConfig->featureMaintainer.minimumDistanceBetweenFeatures = 49.0;
    stereo->rectifiedLeft.link(featureTracker->inputImage);

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
    featureTracker->passthroughInputImage.link(odom->rect);
    stereo->depth.link(odom->depth);
    imu->out.link(odom->imu);
    featureTracker->outputFeatures.link(odom->features);

    odom->transform.link(slam->odom);
    odom->passthroughRect.link(slam->rect);
    odom->passthroughDepth.link(slam->depth);
    // odom->passthroughFeatures.link(slam->inputFeatures);

    slam->transform.link(rerun->inputTrans);
    slam->passthroughRect.link(rerun->inputImg);
    slam->occupancyGridMap.link(rerun->inputMap);
    slam->obstaclePCL.link(rerun->inputObstaclePCL);
    slam->groundPCL.link(rerun->inputGroundPCL);

    pipeline.start();
    pipeline.wait();
}
