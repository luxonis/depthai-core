#include <chrono>
#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main() {
    // Initialize Device
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto cam_left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cam_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto* left_out = cam_left->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* right_out = cam_right->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // Dynamic-calibration node
    auto dyn_calib = pipeline.create<dai::node::DynamicCalibration>();
    left_out->link(dyn_calib->left);
    right_out->link(dyn_calib->right);

    // O/I queues
    auto calibration_output = dyn_calib->calibrationOutput.createOutputQueue();
    auto coverage_output = dyn_calib->coverageOutput.createOutputQueue();

    auto initial_config_input = dyn_calib->inputConfig.createInputQueue();
    auto command_input = dyn_calib->inputControl.createInputQueue();

    // Get calibration data from device
    device->setCalibration(device->readCalibration());

    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));  // wait for autoexposure to settle

    // start loading the collecting data
    command_input->send(std::make_shared<dai::StartRecalibrationCommand>());

    while(pipeline.isRunning()) {
        // wait for a coverage data
        const auto coverage = coverage_output->get<dai::CoverageData>();
        std::cout << "Coverage = " << coverage->meanCoverage << std::endl;
        const auto calibration_result = calibration_output->get<dai::DynamicCalibrationResult>();
        const auto& calibration_data = calibration_result->calibrationData;
        if(calibration_data.has_value()) {
            command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(calibration_data->newCalibration));
            std::cout << "Successfully recalibrated" << std::endl;
            break;
        } else {
            std::cout << calibration_result->info << std::endl;
        }
    }

    return 0;
}
