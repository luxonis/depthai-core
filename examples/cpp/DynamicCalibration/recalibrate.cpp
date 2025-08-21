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
    int number_of_saved_pics = 5;

    // Create output directory
    fs::create_directories(folder);

    // Initialize Device
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------


    dai::Pipeline pipeline(device);

    // Create camera nodes
    auto cam_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);


    auto cam_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);



    //auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto* left_out  = cam_left->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* right_out = cam_right->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // Dynamic-calibration node
    auto dyn_calib = pipeline.create<dai::node::DynamicCalibration>();
    // auto left_queue = left_out->createOutputQueue();
    // auto right_queue = rightOut->createOutputQueue();
    left_out->link(dyn_calib->left);
    right_out->link(dyn_calib->right);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    left_out->link(stereo->left);
    right_out->link(stereo->right);
    auto left_xout = stereo->syncedLeft.createOutputQueue();
    auto right_xout = stereo->syncedRight.createOutputQueue();
    auto disp_xout = stereo->disparity.createOutputQueue();

	// O/I queues
    auto calibration_output = dyn_calib->calibrationOutput.createOutputQueue();
    auto coverage_output = dyn_calib->coverageOutput.createOutputQueue();

    auto initial_config_input = dyn_calib->configInput.createInputQueue();
    auto command_input = dyn_calib->commandInput.createInputQueue();

    // start loading the collecting data

    auto calib_data = device->readCalibration();
	device->setCalibration(calib_data);
    nlohmann::json calib_json = calib_data.eepromToJson();
    std::ofstream before_calib_file(folder + "calibration_before.json");
    before_calib_file << calib_json.dump(4);
    before_calib_file.close();

    // Wait for autoexposure to settle
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Send recalibration command
    command_input->send(std::make_shared<dai::StartRecalibrationCommand>(dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE));


    int iteration = 0;
    while (pipeline.isRunning()) {
        iteration++;
        std::cout << "Iteration " << iteration << " ... " << std::endl;

        // Wait for coverage data
        auto coverage = coverage_output->get<dai::CoverageData>();
        std::cout << "Coverage = " << coverage->meanCoverage << std::endl;

        // Wait for calibration result
        auto calibration_result = calibration_output->get<dai::DynamicCalibrationResult>();

        if (calibration_result->calibrationData.has_value()) {
            // Save a few images
            for (int i = 0; i < number_of_saved_pics; ++i) {
                auto in_left = left_xout->get<dai::ImgFrame>();
                auto in_right = right_xout->get<dai::ImgFrame>();
                auto left_data = in_left->getData();
                auto right_data = in_right->getData();

                std::ofstream left_img(folder + "img_left_" + std::to_string(i) + ".bin", std::ios::binary);
                left_img.write(reinterpret_cast<const char*>(left_data.data()), left_data.size());
                left_img.close();
                std::ofstream right_img(folder + "img_right_" + std::to_string(i) + ".bin", std::ios::binary);
                right_img.write(reinterpret_cast<const char*>(right_data.data()), right_data.size());
                right_img.close();
            }

            // Save calibration after
            nlohmann::json new_calib_json = calibration_result->calibrationData->newCalibration.eepromToJson();
            std::ofstream new_calib_file(folder + "calibration_after.json");
            new_calib_file << new_calib_json.dump(4);
            new_calib_file.close();

            std::cout << "Successfully recalibrated" << std::endl;

            // Apply new calibration
            command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(calibration_result->calibrationData->newCalibration));
            break;
        } else {
            std::cout << calibration_result->info << std::endl;
        }
    }
    return 0;
}
