#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <thread>
#include <vector>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

int main() {
    std::string folder = "data/sessionXYZ/";

    fs::create_directories(folder);

    // Create and initialize Device
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    // Create camera nodes
    auto cam_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cam_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    // Request full resolution NV12 outputs
    auto* left_out  = cam_left->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* right_out = cam_right->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // Stereo node
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    left_out->link(stereo->left);
    right_out->link(stereo->right);

    // Dynamic calibration node
    auto dyn_calib = pipeline.create<dai::node::DynamicCalibration>();
    left_out->link(dyn_calib->left);
    right_out->link(dyn_calib->right);

    // Output queues
    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

    auto calibration_output = dyn_calib->calibrationOutput.createOutputQueue();
    auto coverage_output = dyn_calib->coverageOutput.createOutputQueue();
    auto quality_output = dyn_calib->qualityOutput.createOutputQueue();

    auto command_input = dyn_calib->commandInput.createInputQueue();
    auto initial_config_input = dyn_calib->configInput.createInputQueue();

    // Set config
    auto config = std::make_shared<dai::DynamicCalibrationConfig>();
    config->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
    config->loadImagePeriod = 0.5f;
    initial_config_input->send(config);

    // Read and set calibration
    auto calibration = device->readCalibration();
    auto old_calibration = calibration;
    std::shared_ptr<dai::CalibrationHandler> new_calibration;
    device->setCalibration(calibration);

    // Save calibration before
    nlohmann::json calib_json = calibration.eepromToJson();
    std::ofstream before_calib_file(folder + "calibration_before.json");
    before_calib_file << calib_json.dump(4);
    before_calib_file.close();

    // Start pipeline
    pipeline.start();

    while (pipeline.isRunning()) {
        int max_disp = stereo->initialConfig->getMaxDisparity();

        auto in_left = left_xout->get<dai::ImgFrame>();
        auto in_right = right_xout->get<dai::ImgFrame>();
        auto in_disp = disp_xout->get<dai::ImgFrame>();

        auto disp_frame = in_disp->getFrame();
        cv::Mat disp_vis;
        disp_frame.convertTo(disp_vis, CV_8U, 255.0 / max_disp);
        cv::applyColorMap(disp_vis, disp_vis, cv::COLORMAP_JET);

        cv::imshow("Disparity", disp_vis);
        int key = cv::waitKey(1);

        auto coverage = coverage_output->tryGet<dai::CoverageData>();
        if(coverage) {
            std::cout << "Coverage A: ";
//            for(const auto& v : coverage->coveragePerCellA) std::cout << v << " ";
            std::cout << "\nCoverage B: ";
//            for(const auto& v : coverage->coveragePerCellB) std::cout << v << " ";
            std::cout << "\nMean coverage: " << coverage->meanCoverage << "\n";
            std::cout << "Data acquired: " << coverage->dataAcquired << std::endl;
        }

        auto calibration_result = calibration_output->tryGet<dai::DynamicCalibrationResult>();
        if(calibration_result) {
            if(calibration_result->calibrationData.has_value()) {
                std::cout << "Found new calibration" << std::endl;
                new_calibration = std::make_shared<dai::CalibrationHandler>(calibration_result->calibrationData->newCalibration);

                nlohmann::json new_calib_json = calibration_result->calibrationData->newCalibration.eepromToJson();
                std::ofstream new_calib_file(folder + "calibration_after.json");
                new_calib_file << new_calib_json.dump(4);
                new_calib_file.close();
            } else {
                std::cout << calibration_result->info << std::endl;
            }
        }

        if(key == 'q') {
            break;
        }
        if(key == 'r') {
            std::cout << "Recalibrating ..." << std::endl;
            command_input->send(std::make_shared<dai::StartRecalibrationCommand>());
        }
        if(key == 'l') {
            std::cout << "Loading image ..." << std::endl;
            command_input->send(std::make_shared<dai::LoadImageCommand>());
        }
        if(key == 'n') {
            if(new_calibration) {
                std::cout << "Applying new calibration ..." << std::endl;
                command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(*new_calibration));
                old_calibration = calibration;
                calibration = *new_calibration;
                new_calibration.reset();
            }
        }
        if(key == 'p') {
            if(!old_calibration.eepromToJson().empty()) {
                std::cout << "Applying previous calibration ..." << std::endl;
                command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(old_calibration));
                new_calibration = std::make_shared<dai::CalibrationHandler>(calibration);
                calibration = old_calibration;
                // To avoid repeated application, reset old_calibration if needed
            }
        }
        if(key == 'c') {
            std::cout << "Checking quality ..." << std::endl;
            command_input->send(std::make_shared<dai::CalibrationQualityCommand>(true));
            auto quality_result = quality_output->get<dai::CalibrationQuality>();
    		const auto& quality_data = quality_result->data;
            if(quality_data.has_value()) {
                const auto quality_data_value = quality_data.value();
                const auto& rot = quality_data_value.rotationChange;
                std::cout << "|| r_current - r_new || = "
                          << std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]) << " deg" << std::endl
                          << "mean Sampson error achievable = " << quality_data->sampsonErrorNew << " px\n"
                          << "mean Sampson error current = " << quality_data->sampsonErrorCurrent << " px" << std::endl;
            }
        }
    }
    return 0;
}