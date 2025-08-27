// examples/cpp/DynamicCalibration/calibrate.cpp
#include <chrono>
#include <iostream>
#include <thread>

#include "depthai/depthai.hpp"

int main() {
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto cam_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cam_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto* leftOut = cam_left->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* rightOut = cam_right->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // Dynamic-calibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    // In-pipeline host queues
    auto leftQ = stereo->syncedLeft.createOutputQueue();
    auto rightQ = stereo->syncedRight.createOutputQueue();
    auto dispQ = stereo->disparity.createOutputQueue();

    auto calibOutQ = dynCalib->calibrationOutput.createOutputQueue();
    auto coverageOutQ = dynCalib->coverageOutput.createOutputQueue();

    auto cfgInQ = dynCalib->inputConfig.createInputQueue();
    auto cmdInQ = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    // Start calibration (optimize performance)
    {
        auto startCmd = std::make_shared<dai::StartCalibrationCommand>();
        startCmd->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
        cmdInQ->send(startCmd);
    }

    int iteration = 0;

    while(pipeline.isRunning()) {
        iteration++;
        std::cout << "Iteration " << iteration << " ... " << std::endl;

        // Wait for coverage info
        if(auto coverageMsg = coverageOutQ->get<dai::CoverageData>()) {
            std::cout << "Coverage = " << coverageMsg->meanCoverage << std::endl;
        }

        // Wait for calibration result
        auto calibMsg = calibOutQ->get<dai::DynamicCalibrationResult>();
        if(!calibMsg) {
            std::cout << "No calibration message received." << std::endl;
            continue;
        }

        // If calibration succeeded, apply it to the device
        if(calibMsg->calibrationData) {
            std::cout << "Successfully calibrated." << std::endl;

            auto applyCmd = std::make_shared<dai::ApplyCalibrationCommand>();
            // 4) field is 'calibration' (the new cal blob)
            applyCmd->calibration = calibMsg->calibrationData->newCalibration;
            cmdInQ->send(applyCmd);
            break;
        }
    }

    return 0;
}
