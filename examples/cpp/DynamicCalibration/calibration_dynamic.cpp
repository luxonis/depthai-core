// examples/cpp/DynamicCalibration/calibrate.cpp
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

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    // In-pipeline host queues
    auto leftSyncedQueue = stereo->syncedLeft.createOutputQueue();
    auto rightSyncedQueue = stereo->syncedRight.createOutputQueue();
    auto depthQueue = stereo->depth.createOutputQueue();

    auto dynCalibOutQ = dynCalib->calibrationOutput.createOutputQueue();
    auto dynCoverageOutQ = dynCalib->coverageOutput.createOutputQueue();

    auto dynCalibInputControl = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->getCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    using DCC = dai::DynamicCalibrationControl;
    // Optionally set performance mode:
    dynCalibInputControl->send(DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_PERFORMANCE));

    // Start calibration (optimize performance)
    dynCalibInputControl->send(DCC::startCalibration());

    while(pipeline.isRunning()) {
        auto leftSynced = leftSyncedQueue->get<dai::ImgFrame>();
        auto rightSynced = rightSyncedQueue->get<dai::ImgFrame>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        cv::imshow("left", leftSynced->getCvFrame());
        cv::imshow("right", rightSynced->getCvFrame());

        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth).getCvFrame());

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

                if(!q.pairwiseRotationDifference.empty()) {
                    const auto& pairwiseRotation = q.pairwiseRotationDifference.begin()->second;
                    float rotDiff = 0.0f;
                    for(const auto axis : pairwiseRotation) rotDiff += axis * axis;
                    rotDiff = std::sqrt(rotDiff);
                    std::cout << "Rotation difference: " << rotDiff << " deg\n";
                }
                std::cout << "Mean Sampson error achievable = " << q.sampsonErrorNew << " px\n";
                std::cout << "Mean Sampson error current    = " << q.sampsonErrorCurrent << " px\n";

                // Reset and start a new round if desired
                dynCalibInputControl->send(DCC::startCalibration());
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q') break;
    }

    return 0;
}
