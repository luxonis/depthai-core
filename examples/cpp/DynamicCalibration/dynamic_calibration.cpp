#include <depthai/depthai.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <string>
#include <getopt.h>


void overlayCoverageOnGray(cv::Mat& grayImage, const std::vector<std::vector<float>>& coveragePerCellB) {
    // Convert grayscale to BGR so we can overlay color
    cv::Mat colorImage;
    cv::cvtColor(grayImage, colorImage, cv::COLOR_GRAY2BGR);

    int rows = coveragePerCellB.size();
    int cols = coveragePerCellB[0].size();
    int cellWidth = colorImage.cols / cols;
    int cellHeight = colorImage.rows / rows;

    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float coverage = coveragePerCellB[y][x];
            if (coverage <= 0.0f) continue;

            cv::Rect cellRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
            cv::Mat roi = colorImage(cellRect);

            float alpha = 0.5f * std::min(1.0f, std::max(0.0f, coverage));  // Clamp [0,1]
            cv::Scalar green(0, 255, 0);  // BGR

            cv::Mat coloredOverlay(cellRect.height, cellRect.width, roi.type(), green);
            cv::addWeighted(coloredOverlay, alpha, roi, 1.0 - alpha, 0.0, roi);
        }
    }

    // Replace original with colored image
    grayImage = colorImage;
}

int main(int argc, char** argv) {
    std::string ip_address;
    int opt;
    while((opt = getopt(argc, argv, "i:")) != -1) {
        switch(opt) {
            case 'i':
                ip_address = optarg;
                break;
        }
    }

    // Initialize Device with optional IP address
    std::shared_ptr<dai::Device> device;
    if(!ip_address.empty()) {
        device = std::make_shared<dai::Device>(ip_address);
    } else {
        device = std::make_shared<dai::Device>();
    }

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
    auto dyncal_out = dynCalib->outputCalibrationResults.createOutputQueue();
    auto input_config = dynCalib->inputConfig.createInputQueue();
    bool continious = false;
    if (continious) {
        dynCalib->setPerformanceMode(dai::DynamicCalibrationConfig::AlgorithmControl::PerformanceMode::DEFAULT);
        dynCalib->setContiniousMode();
        dynCalib->setTimeFrequency(4);
    }
    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);
    // Get calibration data from device
    auto calibOld = device->readCalibration();
    auto calibNew = device->readCalibration();
    device->setCalibration(calibOld);

    pipeline.start();

    auto lastCalibrationTime = std::chrono::steady_clock::now();
    const auto calibrationInterval = std::chrono::seconds(1); // 1 second interval
    while(pipeline.isRunning()) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto leftFrameQueue = leftQueue->get<dai::ImgFrame>();
        auto rightFrameQueue = rightQueue->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        frame.convertTo(frame, CV_8UC1, 255 / stereo->initialConfig->getMaxDisparity());
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);

        if(leftFrameQueue == nullptr || rightFrameQueue == nullptr ) continue;
        auto left_frame = leftFrameQueue->getCvFrame();
        auto right_frame = rightFrameQueue->getCvFrame();

        // std::cout << qualityResult.info << " " << qualityResult.valid << " " << qualityResult.value << std::endl;
        auto calibration_result = dyncal_out->tryGet();
        auto dynResult = std::dynamic_pointer_cast<dai::DynamicCalibrationResults>(calibration_result);
        if(dynResult && dynResult->newCalibration->calibHandler.has_value()) {
            calibNew = *dynResult->newCalibration->calibHandler;
            std::cout << "Got new calibration. " << std::endl;
        }
        if(dynResult && dynResult->calibOverallQuality.has_value() && dynResult->calibOverallQuality->report) {
            double meanCoverage = dynResult->calibOverallQuality->report->coverageQuality->meanCoverage;
            auto coveragePerCellB = dynResult->calibOverallQuality->report->coverageQuality->coveragePerCellB;
            overlayCoverageOnGray(left_frame, coveragePerCellB);
            auto& report = dynResult->calibOverallQuality->report;
            if(report.has_value() && report->calibrationQuality.has_value()) {
                auto& rotationChange = report->calibrationQuality->rotationChange;

                std::cout << "Rotation change (as float): ";
                for(const auto& val : rotationChange) {
                    std::cout << static_cast<float>(val) << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "No calibrationQuality present." << std::endl;
            }
            std::cout << "Got calibCheck. Coverage quality = " << meanCoverage << std::endl;
        }

        cv::imshow("left", left_frame);
        cv::imshow("right", right_frame);
        cv::imshow("disparity_color", frame);
        auto key = cv::waitKey(1);

        if(key == 'q') {
            break;
        }
        else if(key == 'o'){
            device->setCalibration(calibOld);
            std::cout << "Applying old calibration " << std::endl;
        }
        else if(key == 'n'){
            device->setCalibration(calibNew);
            std::cout << "Applying new calibration " << std::endl;
        }
        else if(key == 'c') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand =  dai::DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK;
            input_config->send(configMessage);
            std::cout << "Start calib check"  << std::endl;
        }
        else if(key == 'r') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION;
            input_config->send(configMessage);
            std::cout << "Start new calibration" << std::endl;
        }
        else if(key == 'C') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand =  dai::DynamicCalibrationConfig::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK;
            input_config->send(configMessage);
            std::cout << "Start forced calib check" << std::endl;
        }
        else if(key == 'R') {
            auto configMessage = std::make_shared<dai::DynamicCalibrationConfig>();
            configMessage->calibrationCommand = dai::DynamicCalibrationConfig::CalibrationCommand::START_FORCE_RECALIBRATION;
            input_config->send(configMessage);
            std::cout << "Start forced new calibration" << std::endl;
        }
    }
    return 0;
}