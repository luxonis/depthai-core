// examples/cpp/DynamicCalibration/calibrate.cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

int main() {
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto* leftOut = monoLeft->requestFullResolutionOutput();
    auto* rightOut = monoRight->requestFullResolutionOutput();

    // Dynamic-calibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    auto dynCalibOutQ = dynCalib->calibrationOutput.createOutputQueue();
    auto dynCoverageOutQ = dynCalib->coverageOutput.createOutputQueue();

    auto dynCalibInputControl = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    using DCC = dai::DynamicCalibrationControl;
    // Optionally set performance mode:
    dynCalibInputControl->send(DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_PERFORMANCE));

    // Start calibration (optimize performance)
    dynCalibInputControl->send(DCC::startCalibration());

    double maxDisparity = 1.0;
    while(pipeline.isRunning()) {
        // Coverage (non-blocking)
        if(auto coverageMsg = dynCoverageOutQ->tryGet<dai::CoverageData>()) {
            std::cout << "2D Spatial Coverage = " << coverageMsg->meanCoverage << "  / 100 [%]\n";
            std::cout << "Data Acquired       = " << coverageMsg->dataAcquired << "  / 100 [%]\n";
        }

        // Calibration result (non-blocking)
        if(auto dynCalibrationResult = dynCalibOutQ->tryGet<dai::DynamicCalibrationResult>()) {
            std::cout << "Dynamic calibration status: " << dynCalibrationResult->info << std::endl;

            if(dynCalibrationResult->calibrationData) {
                std::cout << "Successfully calibrated." << std::endl;

                // Apply the produced calibration
                const auto& newCalib = dynCalibrationResult->calibrationData->newCalibration;
                dynCalibInputControl->send(DCC::applyCalibration(newCalib));

                // Print quality deltas
                const auto& q = dynCalibrationResult->calibrationData->calibrationDifference;

                float rotDiff = std::sqrt(q.rotationChange[0] * q.rotationChange[0] + q.rotationChange[1] * q.rotationChange[1]
                                          + q.rotationChange[2] * q.rotationChange[2]);
                std::cout << "Rotation difference: " << rotDiff << " deg\n";
                std::cout << "Mean Sampson error achievable = " << q.sampsonErrorNew << " px\n";
                std::cout << "Mean Sampson error current    = " << q.sampsonErrorCurrent << " px\n";
                std::cout << "Theoretical Depth Error Difference "
                          << "@1m:" << std::fixed << std::setprecision(2) << q.depthErrorDifference[0] << "%, "
                          << "2m:" << q.depthErrorDifference[1] << "%, "
                          << "5m:" << q.depthErrorDifference[2] << "%, "
                          << "10m:" << q.depthErrorDifference[3] << "%\n";

                // Reset and start a new round if desired
                dynCalibInputControl->send(DCC::startCalibration());
            }
        }
    }

    return 0;
}
