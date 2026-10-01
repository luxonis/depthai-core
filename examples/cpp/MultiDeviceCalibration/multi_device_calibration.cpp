// Cross-device calibration between two or more devices in ONE dai::Pipeline.
//
// Each device contributes its factory-calibrated CAM_B/CAM_C stereo pair to a
// host MultiDeviceCalibration node. The node collects synchronized image groups,
// estimates the metric extrinsics between the devices and emits a pure-data
// MultiDeviceCalibrationResult that is saved to a JSON file.
//
// Usage: multi_device_calibration [-d|--devices <device_1> <device_2> ...]
//                                 [-n|--sample-count <count>] [-o|--output <path>]

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

struct ParsedArgs {
    std::vector<std::string> deviceArgs;
    std::size_t sampleCount = 10;
    std::filesystem::path output = "multi_device_calibration.json";
};

std::optional<ParsedArgs> parseArguments(int argc, char** argv) {
    ParsedArgs parsed;
    auto printUsage = [argv]() {
        std::cout << "Usage: " << argv[0] << " [-d|--devices <device_1> <device_2> ...] [-n|--sample-count <count>] [-o|--output <path>]" << std::endl;
    };
    try {
        for(int i = 1; i < argc;) {
            const std::string arg = argv[i];
            if(arg == "-d" || arg == "--devices") {
                i++;
                while(i < argc && argv[i][0] != '-') {
                    parsed.deviceArgs.emplace_back(argv[i++]);
                }
                if(parsed.deviceArgs.size() < 2) throw std::runtime_error("Option " + arg + " requires at least two devices");
            } else if(arg == "-n" || arg == "--sample-count") {
                if(i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
                parsed.sampleCount = static_cast<std::size_t>(std::stoul(argv[i + 1]));
                i += 2;
            } else if(arg == "-o" || arg == "--output") {
                if(i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
                parsed.output = argv[i + 1];
                i += 2;
            } else if(arg == "-h" || arg == "--help") {
                printUsage();
                return std::nullopt;
            } else {
                throw std::runtime_error("Unknown option: " + arg);
            }
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        printUsage();
        return std::nullopt;
    }
    return parsed;
}

}  // namespace

int main(int argc, char** argv) {
    auto parsed = parseArguments(argc, argv);
    if(!parsed.has_value()) return 1;

    std::vector<dai::DeviceInfo> deviceInfos;
    if(parsed->deviceArgs.empty()) {
        // Default to the first two available devices
        deviceInfos = dai::Device::getAllAvailableDevices();
        if(deviceInfos.size() > 2) deviceInfos.resize(2);
        if(deviceInfos.size() < 2) {
            std::cout << "At least two devices are required for this example." << std::endl;
            return 0;
        }
    } else {
        for(const auto& arg : parsed->deviceArgs) deviceInfos.emplace_back(arg);
    }

    const std::vector<dai::CameraBoardSocket> sockets{dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C};
    constexpr float fps = 5.0f;

    // One pipeline without an implicit device; every device is added explicitly
    dai::Pipeline pipeline(false);

    auto calibration = pipeline.create<dai::beta::node::MultiDeviceCalibration>();
    calibration->setSampleCount(parsed->sampleCount);
    calibration->sync->setSyncThreshold(std::chrono::seconds(5));

    for(const auto& info : deviceInfos) {
        auto device = pipeline.addDevice(info);
        const auto deviceId = device->getDeviceId();
        std::cout << "Using device " << deviceId << std::endl;

        // Two cameras per device give the solver a factory-calibrated baseline, which fixes the metric scale.
        // The device ID and socket are taken from the Camera node that owns the output.
        for(auto socket : sockets) {
            auto camera = pipeline.create<dai::node::Camera>(device)->build(socket, std::nullopt, fps);
            calibration->addCamera(*camera->requestFullResolutionOutput(std::nullopt, fps));
        }
    }

    auto controlQueue = calibration->inputControl.createInputQueue();
    auto resultQueue = calibration->calibrationOutput.createOutputQueue();

    std::cout << "Point all devices at the same textured scene and keep them still." << std::endl;
    pipeline.start();
    controlQueue->send(dai::beta::MultiDeviceCalibrationControl::start());

    bool hasTimedOut = false;
    auto result = resultQueue->get<dai::beta::MultiDeviceCalibrationResult>(std::chrono::minutes(3), hasTimedOut);
    if(result == nullptr) {
        throw std::runtime_error("Calibration timed out");
    }
    if(!result->passed || !result->graph.has_value()) {
        throw std::runtime_error(result->info);
    }

    const auto handler = result->getHandler();
    if(!handler.has_value() || !handler->toJsonFile(parsed->output)) {
        throw std::runtime_error("Failed to save calibration to " + parsed->output.string());
    }

    std::cout << "Calibration saved to " << parsed->output.string() << std::endl;
    std::cout << std::fixed << std::setprecision(3) << "Confidence: " << result->dataConfidence << std::endl;
    std::cout << std::defaultfloat << std::setprecision(6) << "Sampson error: " << result->sampsonError << std::endl;

    pipeline.stop();
    return 0;
}
