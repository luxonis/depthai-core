
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
    auto left = pipeline.create<dai::node::MonoCamera>()->build();
    auto right = pipeline.create<dai::node::MonoCamera>()->build();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>()->build();
    auto featureTracker = pipeline.create<dai::node::FeatureTracker>()->build();
    auto odom = pipeline.create<dai::node::RTABMapVIO>()->build();
    auto slam = pipeline.create<dai::node::RTABMapSLAM>()->build();
    auto rerun = pipeline.create<RerunNode>();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), "30"));

    odom->setParams(params);

    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGrid3D(), "true"));
    slam->setParams(params);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig.setNumTargetFeatures(1000);
    featureTracker->initialConfig.setMotionEstimator(false);
    featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 49.0;
    stereo->rectifiedLeft.link(featureTracker->inputImage);
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setCamera("right");
    right->setFps(fps);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(true);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
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
