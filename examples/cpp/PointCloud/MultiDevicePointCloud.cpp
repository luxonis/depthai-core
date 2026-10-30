// Multi-device PointCloud: one merged cloud from the depth of several devices.
//
// Every device computes depth aligned to its color camera. Each depth + color pair is linked to the
// same PointCloud node through getDepthInput(<device id>) / getColorInput(<device id>); the node
// synchronizes the streams and merges them into a single PointCloudData expressed in the common
// origin of a multi-device calibration (see README.md, "Merging several depth streams").
//
// Usage: multi_device_point_cloud [-d|--devices <device_1> <device_2> ...] [-c|--calibration <path>]
//                                 [--size <width> <height>] [--target-device <id>]
//                                 [--target-socket <CAM_A..CAM_J> | --target-housing <housing>]
//
// The merged cloud is shown in the DepthAI visualizer: open http://localhost:8082 and press Q to quit.

#include <chrono>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"

namespace {

constexpr float FPS = 10.0f;

struct ParsedArgs {
    std::vector<std::string> deviceArgs;
    std::filesystem::path calibration = "multi_device_calibration.json";
    std::pair<uint32_t, uint32_t> size{640, 400};
    std::optional<std::string> targetDevice;
    dai::CameraBoardSocket targetSocket = dai::CameraBoardSocket::CAM_A;
    std::optional<dai::HousingCoordinateSystem> targetHousing;
};

// Enumerator of `Enum` in [first, last] whose dai::toString matches `text`
template <typename Enum>
Enum parseEnum(const std::string& text, Enum first, Enum last) {
    for(int value = static_cast<int>(first); value <= static_cast<int>(last); ++value) {
        if(dai::toString(static_cast<Enum>(value)) == text) return static_cast<Enum>(value);
    }
    throw std::runtime_error("Unknown value: " + text);
}

std::optional<ParsedArgs> parseArguments(int argc, char** argv) {
    ParsedArgs parsed;
    auto printUsage = [argv]() {
        std::cout << "Usage: " << argv[0]
                  << " [-d|--devices <device_1> <device_2> ...] [-c|--calibration <path>] [--size <width> <height>]"
                     " [--target-device <id>] [--target-socket <CAM_A..CAM_J> | --target-housing <housing>]"
                  << std::endl;
    };
    try {
        for(int i = 1; i < argc;) {
            const std::string arg = argv[i];
            auto value = [&](int offset = 1) -> std::string {
                if(i + offset >= argc) throw std::runtime_error("Missing value for " + arg);
                return argv[i + offset];
            };
            if(arg == "-d" || arg == "--devices") {
                i++;
                while(i < argc && argv[i][0] != '-') parsed.deviceArgs.emplace_back(argv[i++]);
                if(parsed.deviceArgs.size() < 2) throw std::runtime_error("Option " + arg + " requires at least two devices");
            } else if(arg == "-c" || arg == "--calibration") {
                parsed.calibration = value();
                i += 2;
            } else if(arg == "--size") {
                parsed.size = {static_cast<uint32_t>(std::stoul(value(1))), static_cast<uint32_t>(std::stoul(value(2)))};
                i += 3;
            } else if(arg == "--target-device") {
                parsed.targetDevice = value();
                i += 2;
            } else if(arg == "--target-socket") {
                parsed.targetSocket = parseEnum(value(), dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_J);
                i += 2;
            } else if(arg == "--target-housing") {
                parsed.targetHousing = parseEnum(value(), dai::HousingCoordinateSystem::CAM_A, dai::HousingCoordinateSystem::IMU);
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

volatile std::sig_atomic_t isRunning = 1;
void signalHandler(int) {
    isRunning = 0;
}

}  // namespace

int main(int argc, char** argv) {
    auto parsed = parseArguments(argc, argv);
    if(!parsed.has_value()) return 1;
    std::signal(SIGINT, signalHandler);

    std::vector<dai::DeviceInfo> deviceInfos;
    if(parsed->deviceArgs.empty()) {
        // Default to the first two available devices
        deviceInfos = dai::Device::getAllAvailableDevices();
        if(deviceInfos.size() > 2) deviceInfos.resize(2);
    } else {
        for(const auto& arg : parsed->deviceArgs) deviceInfos.emplace_back(arg);
    }
    if(deviceInfos.size() < 2) {
        std::cerr << "At least two devices are required for this example." << std::endl;
        return 1;
    }
    if(!std::filesystem::is_regular_file(parsed->calibration)) {
        std::cerr << "Multi-device calibration not found: " << parsed->calibration << std::endl;
        return 1;
    }
    const dai::beta::MultiDeviceCalibrationHandler calibration(parsed->calibration);

    // One pipeline without an implicit device; every device is added explicitly
    dai::Pipeline pipeline(false);
    pipeline.setMultiDeviceCalibration(calibration.getGraph());

    // One PointCloud node merges every device's depth stream
    auto pc = pipeline.create<dai::node::PointCloud>();
    pc->initialConfig->setLengthUnit(dai::LengthUnit::METER);
    pc->sync->setSyncThreshold(std::chrono::milliseconds(static_cast<int>(1000 / FPS)));  // one frame period: the devices are not hardware-synchronized
    if(parsed->targetDevice.has_value()) {
        // Express the merged cloud in a coordinate system of one particular device. Frames of the other
        // devices are carried over through the multi-device calibration and that device's own calibration.
        if(parsed->targetHousing.has_value()) {
            pc->setTargetCoordinateSystem(*parsed->targetDevice, *parsed->targetHousing);
        } else {
            pc->setTargetCoordinateSystem(*parsed->targetDevice, parsed->targetSocket);
        }
    }

    for(const auto& info : deviceInfos) {
        auto device = pipeline.addDevice(info);
        const auto deviceId = device->getDeviceId();

        const auto colorSockets = device->getConnectedCameras(dai::CameraSensorType::COLOR);
        const auto colorSocket = colorSockets.empty() ? dai::CameraBoardSocket::CAM_A : colorSockets.front();
        auto color = pipeline.create<dai::node::Camera>(device)->build(colorSocket, std::nullopt, FPS);
        auto* colorOut = color->requestOutput(parsed->size, dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, FPS, true);

        auto depth = pipeline.create<dai::node::Depth>(device)->build(dai::node::Depth::Algorithm::AUTO, FPS, parsed->size);
        depth->setAlignTo(*colorOut);

        depth->depth().link(pc->getDepthInput(deviceId));
        colorOut->link(pc->getColorInput(deviceId));
        std::cout << "Device " << deviceId << ": linked as point cloud stream '" << deviceId << "'" << std::endl;
    }

    auto queue = pc->outputPointCloud.createOutputQueue(4, false);
    // Non-blocking visualizer queue: a slow browser must never stall the PointCloud node
    dai::RemoteConnection remote;
    auto visualizerQueue = remote.addTopic("merged_point_cloud", "3d", 2, false);

    pipeline.start();
    remote.registerPipeline(pipeline);
    std::cout << "Merged point cloud streams:";
    for(const auto& name : pc->getDepthInputNames()) std::cout << " '" << name << "'";
    std::cout << std::endl;
    if(parsed->targetDevice.has_value()) {
        std::cout << "Cloud expressed in " << (parsed->targetHousing ? dai::toString(*parsed->targetHousing) : dai::toString(parsed->targetSocket))
                  << " of device " << *parsed->targetDevice << std::endl;
    }
    std::cout << "Open the visualizer at http://localhost:8082, press 'q' there or Ctrl-C here to quit." << std::endl;

    std::cout << std::fixed << std::setprecision(2);
    while(isRunning != 0 && pipeline.isRunning()) {
        bool timedOut = false;
        auto pcd = queue->get<dai::PointCloudData>(std::chrono::milliseconds(500), timedOut);
        if(pcd == nullptr) continue;
        visualizerQueue->send(pcd);
        std::cout << "Points: " << pcd->getWidth() * pcd->getHeight() << ", color=" << pcd->isColor() << " X=[" << pcd->getMinX() << ", " << pcd->getMaxX()
                  << "] Y=[" << pcd->getMinY() << ", " << pcd->getMaxY() << "] Z=[" << pcd->getMinZ() << ", " << pcd->getMaxZ() << "] m" << std::endl;
        if(remote.waitKey(1) == 'q') break;
    }

    pipeline.stop();
    return 0;
}
