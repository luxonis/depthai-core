#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <string>

static constexpr const bool kEnableContinuousRecalibration = false;

int main() {

    // Initialize Device with optional IP address
    std::shared_ptr<dai::Device> device;
    device = std::make_shared<dai::Device>();

    // ---------- Pipeline definition ----------
    dai::Pipeline pipeline(device);

    auto camLeft  = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto camRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    //auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto* leftOut  = camLeft->requestFullResolutionOutput();
    auto* rightOut = camRight->requestFullResolutionOutput();

    // Dynamic-calibration node
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();

    // Full-resolution NV12 outputs
    auto dynalOut = dynCalib->calibrationOutput.createOutputQueue();

    auto config = std::make_shared<dai::DynamicCalibrationConfig>();
    config->recalibrationMode = dai::DynamicCalibrationConfig::RecalibrationMode::DEFAULT;
    config->performanceMode = dai::DynamicCalibrationConfig::PerformanceMode::OPTIMIZE_PERFORMANCE;
    config->loadImagePeriod = 0.5; // Period of loading images in sec.

    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();

    auto calibrationOutput = dynCalib->calibrationOutput.createOutputQueue();
    auto coverageOutput = dynCalib->coverageOutput.createOutputQueue();

    auto initialConfigInput = dynCalib->configInput.createInputQueue();
    auto commandInput = dynCalib->commandInput.createInputQueue();

    initialConfigInput->send(config);

    //time.sleep(1);
    commandInput->send(std::make_shared<dai::StartRecalibrationCommand>());

    // Feed the frames into the dynamic-calibration block
    leftOut->link(dynCalib->left);
    rightOut->link(dynCalib->right);
    // Get calibration data from device
    auto calib = device->readCalibration();
    auto initialCalibration = calib;
    auto calibrationNew = calib;
    device->setCalibration(initialCalibration);

    pipeline.start();

    while(pipeline.isRunning()) {
        // wait for a coverage data
        const auto coverage = coverageOutput->get<dai::CoverageData>();
        std::cout << "Coverage = " << coverage->meanCoverage << std::endl;
        const auto calibrationData = calibrationOutput->get<dai::DynamicCalibrationResult>();
        const auto& calibration = calibrationData->calibration;
        if (calibration.has_value())
        {
            const auto calibrationHandler = calibration.value();
            commandInput->send(std::make_shared<dai::ApplyCalibrationCommand>(calibrationHandler));
            std::cout << "Successfully recalibrated" << std::endl;
        }
        else
        {
            std::cout << calibrationData->info << std::endl;
        }
    }

    return 0;
}
