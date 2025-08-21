#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>


int main() {

    // Initialize Device
    auto device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto cam_left  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cam_right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    //auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto* left_out  = cam_left->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto* right_out = cam_right->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    // Dynamic-calibration node
    auto dyn_calib = pipeline.create<dai::node::DynamicCalibration>();
    // auto left_queue = left_out->createOutputQueue();
    // auto right_queue = right_out->createOutputQueue();
    left_out->link(dyn_calib->left);
    right_out->link(dyn_calib->right);

	// O/I queues
    auto calibration_output = dyn_calib->calibrationOutput.createOutputQueue();
    auto coverage_output = dyn_calib->coverageOutput.createOutputQueue();

    auto initial_config_input = dyn_calib->configInput.createInputQueue();
    auto command_input = dyn_calib->commandInput.createInputQueue();

    // auto config = std::make_shared<dai::DynamicCalibrationConfig>();
    // config->recalibrationMode = dai::DynamicCalibrationConfig::RecalibrationMode::DEFAULT;
    // config->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
    // config->loadImagePeriod = 0.5; // Period of loading images in sec.
    // initial_config_input->send(config);

    // Get calibration data from device
    device->setCalibration(device->readCalibration());

    pipeline.start();
	std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for autoexposure to settle	

	// start loading the collecting data
    command_input->send(std::make_shared<dai::StartRecalibrationCommand>());

    while(pipeline.isRunning()) {
        // wait for a coverage data
        const auto coverage = coverage_output->get<dai::CoverageData>();
        std::cout << "Coverage = " << coverage->meanCoverage << std::endl;
        const auto calibration_result = calibration_output->get<dai::DynamicCalibrationResult>();
		const auto& calibration_data = calibration_result->calibrationData;
        if (calibration_data.has_value())
        {
            command_input->send(std::make_shared<dai::ApplyCalibrationCommand>(calibration_data->newCalibration));
            std::cout << "Successfully recalibrated" << std::endl;
        }
        else
        {
            std::cout << calibration_result->info << std::endl;
        }
    }

    pipeline.stop();
	pipeline.wait();
	return 0;
}
