#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "depthai/depthai.hpp"

namespace {

void printCoverage(const std::shared_ptr<dai::CoverageData>& coverage) {
    std::cout << "2D Spatial Coverage = " << coverage->meanCoverage << " / 100 [%]\n";
    std::cout << "Data Acquired       = " << coverage->dataAcquired << " / 100 [%]\n";
}

void printCalibrationResult(const std::shared_ptr<dai::DynamicCalibrationResult>& result) {
    std::cout << "Dynamic calibration status: " << result->info << '\n';
    if(!result->calibrationData) return;

    const auto& calibrationData = *result->calibrationData;
    const auto& q = calibrationData.calibrationDifference;

    std::cout << "Successfully calibrated\n";
    std::cout << "Data confidence: " << std::fixed << std::setprecision(3) << calibrationData.dataConfidence << '\n';

    const float rotDiff =
        std::sqrt(q.rotationChange[0] * q.rotationChange[0] + q.rotationChange[1] * q.rotationChange[1] + q.rotationChange[2] * q.rotationChange[2]);
    std::cout << std::setprecision(2) << "Rotation difference: || r_current - r_new || = " << rotDiff << " deg\n";
    std::cout << std::setprecision(3) << "Mean Sampson error achievable = " << q.sampsonErrorNew << " px\n";
    std::cout << "Mean Sampson error current    = " << q.sampsonErrorCurrent << " px\n";

    for(const auto& [socketPair, rotationDelta] : q.pairwiseRotationDifference) {
        std::cout << std::setprecision(3) << "Pairwise rotation difference " << socketPair.first << " -> " << socketPair.second << ": [" << rotationDelta[0]
                  << ", " << rotationDelta[1] << ", " << rotationDelta[2] << "]\n";
    }
}

}  // namespace

int main() {
    using DCC = dai::DynamicCalibrationControl;

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    auto rgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto* rgbOut = rgb->requestOutput(std::make_pair(1280U, 800U), std::nullopt, dai::ImgResizeMode::CROP, 30.0f);
    auto* leftOut = monoLeft->requestFullResolutionOutput();
    auto* rightOut = monoRight->requestFullResolutionOutput();

    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    rgbOut->link(dynCalib->rgb);
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);

    auto rgbQueue = rgbOut->createOutputQueue();
    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();
    auto coverageQueue = dynCalib->coverageOutput.createOutputQueue();
    auto calibrationQueue = dynCalib->calibrationOutput.createOutputQueue();
    auto inputControl = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    inputControl->send(DCC::setPerformanceMode(DCC::PerformanceMode::OPTIMIZE_PERFORMANCE));
    inputControl->send(DCC::startCalibration());

    while(pipeline.isRunning()) {
        cv::imshow("rgb", rgbQueue->get<dai::ImgFrame>()->getCvFrame());
        cv::imshow("left", leftQueue->get<dai::ImgFrame>()->getCvFrame());
        cv::imshow("right", rightQueue->get<dai::ImgFrame>()->getCvFrame());

        if(auto coverage = coverageQueue->tryGet<dai::CoverageData>()) {
            printCoverage(coverage);
        }

        if(auto calibration = calibrationQueue->tryGet<dai::DynamicCalibrationResult>()) {
            printCalibrationResult(calibration);
            if(calibration->calibrationData) {
                inputControl->send(DCC::applyCalibration(calibration->calibrationData->newCalibration));
            }
        }

        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
