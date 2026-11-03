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

// Project cameras from multiple devices onto a plane (bird's-eye view)

static constexpr float FPS = 10.0f;
static const char* WINDOW_NAME = "Calibrated planar projection";

int main(int argc, char** argv) {
    argparse::ArgumentParser program("multi_device_stitching_planar_projection");
    program.add_argument("--devices").required().nargs(argparse::nargs_pattern::at_least_one).help("Two or more device IDs or IPs; one camera per device");
    program.add_argument("--socket").default_value(std::string("CAM_A")).help("Camera used on each device with --devices (CAM_A through CAM_J)");
    program.add_argument("--calibration").required().help("Cross-device calibration JSON");
    program.add_argument("--plane-point")
        .nargs(3)
        .scan<'g', float>()
        .default_value(std::vector<float>{0, 0, 130})
        .help("Plane point X Y Z in the common calibration origin, in centimetres");
    program.add_argument("--plane-normal")
        .nargs(3)
        .scan<'g', float>()
        .default_value(std::vector<float>{0, 1, 1})
        .help("Plane normal X Y Z in the common calibration origin");
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
    const auto point = program.get<std::vector<float>>("--plane-point");
    const auto normal = program.get<std::vector<float>>("--plane-normal");
    if(point.size() != 3 || normal.size() != 3) {
        std::cerr << "--plane-point and --plane-normal each require 3 coordinates\n";
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
        outputs.push_back(camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, FPS));
    }

    auto stitching = pipeline.create<dai::node::Stitching>()->build(outputs);
    stitching->setMode(dai::node::Stitching::Mode::PLANAR_PROJECTION);
    // Both are expressed in the shared calibration origin, with the point in centimetres.
    stitching->setPlane(dai::Point3f(point[0], point[1], point[2]), dai::Point3f(normal[0], normal[1], normal[2]), dai::LengthUnit::CENTIMETER);
    stitching->setMaxRange(2.0f, dai::LengthUnit::METER);
    stitching->setMaxViewSize(1280, 720);
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
