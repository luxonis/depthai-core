
#include <depthai/depthai.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline;

    auto camLeft  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Dynamic-recalibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();

    // Full-resolution NV12 outputs
    auto* leftOut  = camLeft ->requestFullResolutionOutput();
    auto* rightOut = camRight->requestFullResolutionOutput();

    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    pipeline.start();
    while(pipeline.isRunning()) {
        int quality = dynCalib->getCalibrationQuality();
        std::cout << "Calibration quality: " << quality << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}