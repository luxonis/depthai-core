

#include "rerun_node.hpp"

#include "depthai/rtabmap/RTABMapSLAM.hpp"
#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"


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
    auto left = pipeline.create<dai::node::MonoCamera>()->build();
    auto right = pipeline.create<dai::node::MonoCamera>()->build();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto imu = pipeline.create<dai::node::IMU>()->build();
    auto odom = pipeline.create<dai::node::BasaltVIO>()->build();
    auto slam = pipeline.create<dai::node::RTABMapSLAM>()->build();
    auto params = rtabmap::ParametersMap();
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGrid3D(), "true"));
    params.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapSaveWMState(), "true"));
    slam->setParams(params);
    auto rerun = pipeline.create<RerunNode>();

    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    odom->setImuUpdateRate(200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(true);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
	stereo->enableDistortionCorrection(true);
    stereo->initialConfig.setLeftRightCheckThreshold(10);
    stereo->setDepthAlign(dai::StereoDepthProperties::DepthAlign::RECTIFIED_LEFT);

    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setFps(fps);
    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setFps(fps);

    // Linking
    left->out.link(stereo->left);
    right->out.link(stereo->right);
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
