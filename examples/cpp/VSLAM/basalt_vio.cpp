#include "depthai/depthai.hpp"
#include "rerun_node.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto imu = pipeline.create<dai::node::IMU>();
    auto odom = pipeline.create<dai::node::BasaltVIO>();

    auto rerun = pipeline.create<RerunNode>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Linking
    left->requestOutput(std::make_pair(width, height))->link(odom->left);
    right->requestOutput(std::make_pair(width, height))->link(odom->right);
    imu->out.link(odom->imu);
    odom->transform.link(rerun->inputTrans);
    odom->passthrough.link(rerun->inputImg);

    pipeline.start();
    pipeline.wait();
}
