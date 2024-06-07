#include "rerun_node.hpp"

#include "depthai/depthai.hpp"


int main() {
    using namespace std;
    // ULogger::setType(ULogger::kTypeConsole);
	// ULogger::setLevel(ULogger::kDebug);
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
    auto rerun = pipeline.create<RerunStreamer>();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOdomResetCountdown(), "30"));

    odom->setParams(params);
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig.setNumTargetFeatures(1000);
    featureTracker->initialConfig.setMotionEstimator(false);
    featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 49.0;
    // stereo->setAlphaScaling(0.0);
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setCamera("right");
    right->setFps(fps);
	stereo->setExtendedDisparity(false);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->rectifiedLeft.link(featureTracker->inputImage);
    featureTracker->passthroughInputImage.link(odom->rect);
    stereo->depth.link(odom->depth);
    imu->out.link(odom->imu);
    featureTracker->outputFeatures.link(odom->features);
    odom->transform.link(rerun->inputTrans);
    odom->passthroughRect.link(rerun->inputImg);
    pipeline.start();
    pipeline.wait();
}
