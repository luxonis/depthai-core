#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/test/MyConsumer.hpp"
#include "depthai/pipeline/node/test/MyProducer.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"

// shared
#include "depthai/properties/XLinkOutProperties.hpp"

// project
#include "depthai/pipeline/ThreadedNode.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/rtabmap/VIO.hpp"


int main() {
    using namespace std;
    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 30;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::ColorCamera>();
    auto right = pipeline.create<dai::node::ColorCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto featureTracker = pipeline.create<dai::node::FeatureTracker>();
    auto odom = pipeline.create<dai::node::VisualOdometry>();

    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 100);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    featureTracker->setHardwareResources(1, 2);
    featureTracker->initialConfig.setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI);
    featureTracker->initialConfig.setNumTargetFeatures(1000);
    featureTracker->initialConfig.setMotionEstimator(false);
    featureTracker->initialConfig.featureMaintainer.minimumDistanceBetweenFeatures = 50.0;
    stereo->rectifiedLeft.link(featureTracker->inputImage);


    left->setIspScale(9,20);
    left->setCamera("left");
    left->setFps(fps);
    right->setIspScale(9,20);
    right->setCamera("right");
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    // Linking
    left->isp.link(stereo->left);
    right->isp.link(stereo->right);
    featureTracker->passthroughInputImage.link(odom->inputRect);
    stereo->depth.link(odom->inputDepth);
    imu->out.link(odom->inputIMU);
    featureTracker->outputFeatures.link(odom->inputFeatures);
    pipeline.start();
    pipeline.wait();
}
