// examples/cpp/DynamicCalibration/calibrate.cpp
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

namespace {
void printMetrics(const dai::CalibrationQuality::Data& q) {
    for(const auto& [socketPair, rotation] : q.pairwiseRotationDifference) {
        std::cout << "Pairwise rotation difference " << static_cast<int>(socketPair.first) << " -> " << static_cast<int>(socketPair.second) << " = [";
        for(std::size_t i = 0; i < rotation.size(); ++i) {
            if(i > 0) std::cout << ", ";
            std::cout << rotation[i];
        }
        std::cout << "] deg" << std::endl;
    }
    std::cout << "Mean Sampson error achievable = " << q.sampsonErrorNew << " px" << std::endl;
    std::cout << "Mean Sampson error current    = " << q.sampsonErrorCurrent << " px" << std::endl;
}
}  // namespace

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

    auto dynCoverageOutQ = dynCalib->coverageOutput.createOutputQueue();
    auto dynCalibOutQ = dynCalib->calibrationOutput.createOutputQueue();

    auto dynCalibInputControl = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->getCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle
    auto lastSent = std::chrono::steady_clock::now();

    using DCC = dai::DynamicCalibrationControl;

    while(pipeline.isRunning()) {
        auto leftSynced = leftSyncedQueue->get<dai::ImgFrame>();
        auto rightSynced = rightSyncedQueue->get<dai::ImgFrame>();
        auto depth = depthQueue->get<dai::ImgFrame>();

        cv::imshow("left", leftSynced->getCvFrame());
        cv::imshow("right", rightSynced->getCvFrame());

        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth).getCvFrame());
        // Wait for coverage info
        auto coverageMsg = dynCoverageOutQ->tryGet<dai::CoverageData>();
        if(coverageMsg) {
            std::cout << "2D Spatial Coverage = " << coverageMsg->meanCoverage << "  / 100 [%]" << std::endl;
            std::cout << "Data Acquired = " << coverageMsg->dataAcquired << "  / 100 [%]" << std::endl;
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - lastSent);
        if(elapsed.count() >= 3) {
            dynCalibInputControl->send(DCC::loadImage());
            dynCalibInputControl->send(DCC::calibrate(false));
            lastSent = now;
        }

        auto dynCalibrationResult = dynCalibOutQ->tryGet<dai::DynamicCalibrationResult>();

        if(dynCalibrationResult) {
            std::cout << "Dynamic calibration status: " << dynCalibrationResult->info << std::endl;

            if(dynCalibrationResult->calibrationData) {
                const auto& q = dynCalibrationResult->calibrationData->calibrationDifference;
                std::cout << "Successfully evaluated metrics from calibration output." << std::endl;
                printMetrics(q);
                if(std::abs(q.sampsonErrorNew - q.sampsonErrorCurrent) > 0.05f) {
                    std::cout << "Applying new calibration" << std::endl;
                    dynCalibInputControl->send(DCC::applyCalibration(dynCalibrationResult->calibrationData->newCalibration));
                }
                dynCalibInputControl->send(DCC::resetData());
            }
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
