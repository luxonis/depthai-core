// examples/cpp/DynamicCalibration/calibrate.cpp
#include <chrono>
#include <iostream>
#include <thread>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto* leftOut = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* rightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

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
    auto disparityQueue = stereo->disparity.createOutputQueue();

    auto dynCalibOutQ = dynCalib->calibrationOutput.createOutputQueue();
    auto dynCoverageOutQ = dynCalib->coverageOutput.createOutputQueue();

    auto dynCalibInputConfig = dynCalib->inputConfig.createInputQueue();
    auto dynCalibInputControl = dynCalib->inputControl.createInputQueue();

    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    // Start calibration (optimize performance)
    {
        auto startCmd = std::make_shared<dai::StartCalibrationCommand>();
        startCmd->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
        dynCalibInputControl->send(startCmd);
    }

    while(pipeline.isRunning()) {
        
        auto leftSynced = leftSyncedQueue->get<dai::ImgFrame>();
        auto rightSynced = rightSyncedQueue->get<dai::ImgFrame>();
        auto disparity = disparityQueue->get<dai::ImgFrame>();

        cv::imshow("left", leftSynced->getCvFrame());
        cv::imshow("right", rightSynced->getCvFrame());

        cv::Mat npDisparity = disparity->getFrame();

        double minVal, curMax;
        cv::minMaxLoc(npDisparity, &minVal, &curMax);
        maxDisparity = std::max(maxDisparity, curMax);

        // Normalize the disparity image to an 8-bit scale.
        cv::Mat normalized;
        npDisparity.convertTo(normalized, CV_8UC1, 255.0 / maxDisparity);

        cv::Mat colorizedDisparity;
        cv::applyColorMap(normalized, colorizedDisparity, cv::COLORMAP_JET);

        // Set pixels with zero disparity to black.
        colorizedDisparity.setTo(cv::Scalar(0, 0, 0), normalized == 0);

        cv::imshow("disparity", colorizedDisparity);
        // Wait for coverage info
        auto coverageMsg = dynCoverageOutQ->tryGet<dai::CoverageData>();
        if(coverageMsg) {
            std::cout << "2D Spatial Coverage = " << coverageMsg->meanCoverage << "  / 100 [%]" << std::endl;
            std::cout << "Data Acquired = " << coverageMsg->dataAcquired << "  / 100 [%]" << std::endl;
        }

        // Wait for calibration result
        auto dynCalibrationResult = dynCalibOutQ->get<dai::DynamicCalibrationResult>();
        if(!dynCalibrationResult) {
            std::cout << "Dynamic calibration status: " << dynCalibrationResult->info << std::endl;
        }

        // If calibration succeeded, apply it to the device
        if(dynCalibrationResult->calibrationData) {
            std::cout << "Successfully calibrated." << std::endl;

            // Apply the new calibration to the device
            auto applyCmd = std::make_shared<dai::ApplyCalibrationCommand>();
            applyCmd->calibration = dynCalibrationResult->calibrationData->newCalibration;
            dynCalibInputControl->send(applyCmd);

            // Calibration quality metrics
            const auto& q = dynCalibrationResult->calibrationData->calibrationDifference;

            // --- Rotation difference ---
            // Compute magnitude of the rotation delta vector (in degrees)
            float rotDiff = std::sqrt(
                q.rotationChange[0] * q.rotationChange[0] +
                q.rotationChange[1] * q.rotationChange[1] +
                q.rotationChange[2] * q.rotationChange[2]
            );
            std::cout << "Rotation difference: || r_current - r_new || = " 
                    << rotDiff << " deg" << std::endl;

            // --- Sampson error ---
            // Represents geometric reprojection error before/after calibration
            std::cout << "Mean Sampson error achievable = " 
                      << q.sampsonErrorNew << " px" << std::endl;
            std::cout << "Mean Sampson error current    = " 
                      << q.sampsonErrorCurrent << " px" << std::endl;

            // --- Depth error difference ---
            // Theoretical improvement in depth accuracy at various ranges
            std::cout << "Theoretical Depth Error Difference "
                      << "@1m:"  << std::fixed << std::setprecision(2) << q.depthErrorDifference[0] << "%, "
                      << "2m:"   << q.depthErrorDifference[1] << "%, "
                      << "5m:"   << q.depthErrorDifference[2] << "%, "
                      << "10m:"  << q.depthErrorDifference[3] << "%" 
                      << std::endl;
        }
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        }
    }

    return 0;
}
