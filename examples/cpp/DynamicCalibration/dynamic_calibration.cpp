
#include <depthai/depthai.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>

int main() {
    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline;

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
    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);
    auto device = pipeline.getDefaultDevice();
    auto calibOld = device->readCalibration();
    auto calibNew = device->readCalibration();
    pipeline.start();
    while(pipeline.isRunning()) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto leftFrameQueue = leftQueue->get<dai::ImgFrame>();
        auto rightFrameQueue = rightQueue->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        frame.convertTo(frame, CV_8UC1, 255 / stereo->initialConfig->getMaxDisparity());
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);

        if(leftFrameQueue == nullptr || rightFrameQueue == nullptr ) continue;

        cv::imshow("left", leftFrameQueue->getCvFrame());
        cv::imshow("right", rightFrameQueue->getCvFrame());
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

        // std::cout << qualityResult.info << " " << qualityResult.valid << " " << qualityResult.value << std::endl;
        auto calibration_result = dyncal_out->tryGet();
        auto dynResult = std::dynamic_pointer_cast<dai::DynamicCalibrationResults>(calibration_result);
        if(dynResult && dynResult->newCalibration->calibHandler.has_value()) {
            calibNew = *dynResult->newCalibration->calibHandler;
            std::cout << "Got new calibration. " << std::endl;
        }
        if(dynResult && dynResult->calibOverallQuality.has_value()) {
            double meanCoverage = dynResult->calibOverallQuality->report->coverageQuality->meanCoverage;
            auto& report = dynResult->calibOverallQuality->report;
            if(report.has_value() && report->calibrationQuality.has_value()) {
                auto& rotationChange = report->calibrationQuality->rotationChange;
                auto& depthAccuracy = report->calibrationQuality->depthAccuracy;

                std::cout << "Rotation change (as float): ";
                for(const auto& val : rotationChange) {
                    std::cout << static_cast<float>(val) << " ";
                }
                std::cout << std::endl;

                std::cout << "Depth accuracy changes (as float): ";
                for(const auto& val : depthAccuracy) {
                    std::cout << static_cast<float>(val) << " ";
                }
                std::cout << std::endl;
            } else {
                std::cout << "No calibrationQuality present." << std::endl;
            }
            std::cout << "Got calibCheck. Coverage quality = " << meanCoverage << std::endl;
        }
    }
    return 0;
}