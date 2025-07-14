#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <getopt.h>

static constexpr const bool kEnableContinuousRecalibration = false;

namespace {
cv::Mat overlayCoverageOnGray(const cv::Mat& grayImage, const std::vector<std::vector<float>>& coveragePerCellB) {
    cv::Mat colorImage;

    // Convert to BGR if image is grayscale
    if(grayImage.channels() == 1) {
        cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);
    } else {
        colorImage = grayImage.clone();  // Already BGR, just clone
    }

    int rows = static_cast<int>(coveragePerCellB.size());
    if(rows == 0) return colorImage;

    int cols = static_cast<int>(coveragePerCellB[0].size());
    if(cols == 0) return colorImage;

    int cellWidth = colorImage.cols / cols;
    int cellHeight = colorImage.rows / rows;

    for(int y = 0; y < rows; ++y) {
        for(int x = 0; x < cols; ++x) {
            float coverage = coveragePerCellB[y][x];
            if(coverage <= 0.0f) continue;

            cv::Rect cellRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
            cv::Mat roi = colorImage(cellRect);

            float alpha = 0.5f * std::clamp(coverage, 0.0f, 1.0f);
            cv::Scalar green(0, 255, 0);  // BGR

            cv::Mat overlay(cellRect.height, cellRect.width, roi.type(), green);
            cv::addWeighted(overlay, alpha, roi, 1.0 - alpha, 0.0, roi);
        }
    }

    return colorImage;
}
}

int main() {

    // Initialize Device with optional IP address
    std::shared_ptr<dai::Device> device;
    device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto camLeft  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto* leftOut  = camLeft ->requestFullResolutionOutput();
    auto* rightOut = camRight->requestFullResolutionOutput();

    leftOut->link(stereo->left);
    rightOut->link(stereo->right);

    // Dynamic-recalibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
    // Full-resolution NV12 outputs

    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();

    auto q = stereo->disparity.createOutputQueue();
    auto dyncalOut = dynCalib->outputCalibrationResults.createOutputQueue();
    auto inputConfig = dynCalib->inputConfig.createInputQueue();
    if (kEnableContinuousRecalibration) {
        dynCalib->setPerformanceMode(dai::DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::DEFAULT);
        dynCalib->setContiniousMode();
        dynCalib->setTimeFrequency(4);
    }
    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);
    // Get calibration data from device
    auto calib = device->readCalibration();
    auto initialCalibration = calib;
    auto calibrationNew = calib;
    device->setCalibration(initialCalibration);

    pipeline.start();

    auto lastCalibrationTime = std::chrono::steady_clock::now();
    const auto calibrationInterval = std::chrono::seconds(1); // 1 second interval
    while(pipeline.isRunning()) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto leftFrameQueue = leftQueue->get<dai::ImgFrame>();
        auto rightFrameQueue = rightQueue->get<dai::ImgFrame>();
        if(leftFrameQueue == nullptr || rightFrameQueue == nullptr ) continue;
        auto depthFrame = inDepth->getFrame();
        depthFrame.convertTo(depthFrame, CV_8UC1, 255 / stereo->initialConfig->getMaxDisparity());
        cv::applyColorMap(depthFrame, depthFrame, cv::COLORMAP_JET);

        auto leftFrame = leftFrameQueue->getCvFrame();
        auto rightFrame = rightFrameQueue->getCvFrame();

        auto calibrationResult = dyncalOut->tryGet();
        auto dynResult = std::dynamic_pointer_cast<dai::DynamicCalibrationResults>(calibrationResult);
        if(dynResult && dynResult->newCalibration.has_value() && dynResult->newCalibration->calibHandler.has_value()) {
            calibrationNew = dynResult->newCalibration->calibHandler.value();
            std::cout << "Got new calibration." << std::endl;
        }

        if(dynResult && dynResult->calibOverallQuality.has_value()) {
            auto& quality = dynResult->calibOverallQuality.value();
            if(quality.report.has_value()) {
                auto& report = quality.report.value();

                if(report.coverageQuality.has_value()) {
                    double meanCoverage = report.coverageQuality.value().meanCoverage;
                    auto coveragePerCellB = report.coverageQuality.value().coveragePerCellB;
                    leftFrame = overlayCoverageOnGray(leftFrame, coveragePerCellB);
                    std::cout << "Got calibration Check. Coverage quality = " << meanCoverage << std::endl;
                }

                if(report.calibrationQuality.has_value()) {
                    auto& rotationChange = report.calibrationQuality.value().rotationChange;
                    std::cout << "Rotation change (as float): ";
                    for(const auto& val : rotationChange) {
                        std::cout << static_cast<float>(val) << " ";
                    }
                    std::cout << std::endl;
                } else {
                    std::cout << "No calibrationQuality present." << std::endl;
                }
            }
        }

        cv::imshow("left", leftFrame);
        cv::imshow("right", rightFrame);
        cv::imshow("disparity_color", depthFrame);
        auto key = cv::waitKey(1);

        if(key == 'q') {
            break;
        }
        else if(key == 'o'){
            device->setCalibration(initialCalibration);
            std::cout << "Applying old calibration " << std::endl;
        }
        else if(key == 'n'){
            device->setCalibration(calibrationNew);
            std::cout << "Applying new calibration " << std::endl;
        }
        else if(key == 'c') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK;
            inputConfig->send(configMessage);
            std::cout << "Starting calibration check"  << std::endl;
        }
        else if(key == 'r') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION;
            inputConfig->send(configMessage);
            std::cout << "Starting new calibration" << std::endl;
        }
        else if(key == 'C') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK;
            inputConfig->send(configMessage);
            std::cout << "Starting forced calibration check" << std::endl;
        }
        else if(key == 'R') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_FORCE_RECALIBRATION;
            inputConfig->send(configMessage);
            std::cout << "Starting forced recalibration" << std::endl;
        }
    }
    return 0;
}