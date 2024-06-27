#include "rerun_node.hpp"

#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>()->build();
    auto right = pipeline.create<dai::node::MonoCamera>()->build();
    auto imu = pipeline.create<dai::node::IMU>()->build();
    auto odom = pipeline.create<dai::node::BasaltVIO>()->build();

    auto rerun = pipeline.create<RerunNode>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setFps(fps);
    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setFps(fps);

    // Linking

    left->out.link(odom->left);
    right->out.link(odom->right);
    imu->out.link(odom->imu);
    odom->transform.link(rerun->inputTrans);
    odom->passthrough.link(rerun->inputImg);

    pipeline.start();
    pipeline.wait();
}
