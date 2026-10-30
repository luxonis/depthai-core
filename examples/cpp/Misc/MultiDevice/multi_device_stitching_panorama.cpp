#include <argparse/argparse.hpp>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

// Compose a calibrated panorama from multiple devices

static constexpr float FPS = 30.0f;
static const char* WINDOW_NAME = "Calibrated panorama";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("multi_device_stitching_panorama");
    program.add_argument("--devices").required().nargs(argparse::nargs_pattern::at_least_one).help("Two or more device IDs or IPs; one camera per device");
    program.add_argument("--socket").default_value(std::string("CAM_A")).help("Camera used on each device with --devices (CAM_A through CAM_J)");
    program.add_argument("--calibration").required().help("Cross-device calibration JSON");
    program.add_argument("--projection").default_value(std::string("Cylindrical")).help("Perspective, Cylindrical or Equirectangular");
    try {
        program.parse_args(argc, argv);
    } catch(const std::exception& error) {
        std::cerr << error.what() << std::endl;
        std::cerr << program;
        return 1;
    }
    const auto deviceIds = program.get<std::vector<std::string>>("--devices");
    const auto calibrationPath = program.get<std::string>("--calibration");
    const auto socketName = program.get<std::string>("--socket");
    if(deviceIds.size() < 2) {
        std::cerr << "--devices requires at least two devices\n";
        return 1;
    }
    if(socketName.size() != 5 || socketName.substr(0, 4) != "CAM_" || socketName[4] < 'A' || socketName[4] > 'J') {
        std::cerr << "--socket must be CAM_A through CAM_J\n";
        return 1;
    }
    const auto projection = program.get<std::string>("--projection");
    if(projection != "Perspective" && projection != "Cylindrical" && projection != "Equirectangular") {
        std::cerr << "--projection must be Perspective, Cylindrical or Equirectangular\n";
        return 1;
    }
    // Load and validate the graph before connecting to any devices.
    dai::Pipeline pipeline(false);
    const dai::beta::MultiDeviceCalibrationHandler calibration{std::filesystem::path(calibrationPath)};
    pipeline.setMultiDeviceCalibration(calibration.getGraph());
    pipeline.setStopOnDeviceLoss(true);
    const auto socket = static_cast<dai::CameraBoardSocket>(socketName[4] - 'A');
    std::vector<dai::Node::Output*> outputs;
    outputs.reserve(deviceIds.size());
    for(const auto& id : deviceIds) {
        auto device = pipeline.addDevice(dai::DeviceInfo(id));
        std::cout << "Using device " << device->getDeviceId() << '\n';
        auto camera = pipeline.create<dai::node::Camera>(device)->build(socket, std::nullopt, FPS);
        // Calibrated composition needs undistorted inputs.
        outputs.push_back(camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS, true));
    }

    auto stitching = pipeline.create<dai::node::Stitching>()->build(outputs);
    stitching->setMode(dai::node::Stitching::Mode::PANORAMA);
    stitching->setCameraModel(projection == "Perspective"   ? dai::CameraModel::Perspective
                              : projection == "Cylindrical" ? dai::CameraModel::Cylindrical
                                                            : dai::CameraModel::Equirectangular);
    // Uncomment to trade seam quality for throughput.
    // stitching->setSeamFinder(dai::node::Stitching::SeamFinder::NONE);
    // Uncomment to register the images visually instead of using the calibration.
    // stitching->setUseInputCalibration(false);
    stitching->setMaxPanoramaSize(2000, 1000);
    stitching->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(2.0 / FPS)));
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
    while(pipeline.isRunning()) {
        auto message = output->tryGet<dai::ImgFrame>();
        if(message != nullptr) {
            cv::imshow(WINDOW_NAME, message->getCvFrame());
        }
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }
    pipeline.stop();
    cv::destroyAllWindows();
    return 0;
}
