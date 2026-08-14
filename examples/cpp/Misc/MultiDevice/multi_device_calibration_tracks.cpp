// Estimate the rig calibration of a two-device setup from feature tracks, without any initial guess or known distance.
//
// The per-device calibration (intrinsics, distortion, the pose between CAM_B and CAM_C) is factory calibrated and read
// from the device. What is missing in a multi-device setup is the pose *between* the devices. The MultiDeviceCalibration
// node in FEATURE_TRACKS mode estimates it from a shared, textured, static scene:
//   * SIFT four-view tracks are matched within each device's stereo pair and across the two devices,
//   * an essential matrix between the two reference cameras gives the rotation and the translation direction,
//   * the metric scale is recovered by triangulating the same tracks in each device's own stereo pair.
//
// Unlike the DYNAMIC_CALIBRATION method it needs neither setInitialGuess() nor setKnownDistance(), but it does require a
// stereo pair (CAM_B and CAM_C) per device.
//
// The result is written as a rig json, which the stitching examples load with Pipeline::setMultiDeviceCalibration().
//
// Usage: multi_device_calibration_tracks <device_ip_or_id_1> <device_ip_or_id_2> [-s samples] [-r width height]
//        [-f fps] [-o output.json]

#include <array>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    // The stereo pair of every device: seeing the scene from two cameras of known baseline is what fixes the metric scale
    const std::array<dai::CameraBoardSocket, 2> sockets = {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C};

    std::vector<std::string> deviceNames;
    int samples = 20;
    std::pair<uint32_t, uint32_t> resolution = {1280, 800};
    float fps = 5.0f;
    std::string output = "rig_calibration.json";

    for(int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if(arg == "-s" || arg == "--samples") {
            samples = std::stoi(argv[++i]);
        } else if(arg == "-r" || arg == "--resolution") {
            resolution.first = static_cast<uint32_t>(std::stoul(argv[++i]));
            resolution.second = static_cast<uint32_t>(std::stoul(argv[++i]));
        } else if(arg == "-f" || arg == "--fps") {
            fps = std::stof(argv[++i]);
        } else if(arg == "-o" || arg == "--output") {
            output = argv[++i];
        } else if(!arg.empty() && arg[0] == '-') {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        } else {
            deviceNames.push_back(arg);
        }
    }

    if(deviceNames.size() != 2) {
        std::cerr << "Usage: " << argv[0] << " <device_ip_or_id_1> <device_ip_or_id_2> [-s samples] [-r width height] [-f fps] [-o output.json]\n"
                  << "The first device is the reference the rig is expressed in." << std::endl;
        return 1;
    }

    std::vector<std::shared_ptr<dai::Device>> devices;
    for(const auto& name : deviceNames) devices.push_back(std::make_shared<dai::Device>(dai::DeviceInfo(name)));

    dai::Pipeline pipeline(false);

    std::vector<std::pair<dai::CoordinateFrame, dai::Node::Output*>> sources;
    for(const auto& device : devices) {
        for(const auto socket : sockets) {
            auto camera = pipeline.create<dai::node::Camera>(device)->build(socket);
            auto* output = camera->requestOutput(resolution, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps);
            sources.emplace_back(dai::CoordinateFrame(device->getDeviceId(), socket), output);
        }
    }

    auto calibration = pipeline.create<dai::node::MultiDeviceCalibration>()->build(sources);
    calibration->setMethod(dai::node::MultiDeviceCalibration::Method::FEATURE_TRACKS);
    calibration->setSampleCount(static_cast<size_t>(samples));
    // Free running cameras are not hardware synced, so allow a group to span a couple of frame intervals
    calibration->sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(2.0 / fps)));

    auto rigQueue = calibration->rigCalibration.createOutputQueue();

    pipeline.start();
    std::cout << "Estimating the rig of " << devices.size() << " devices from " << samples
              << " image sets - keep a textured, static scene in view of both of them" << std::endl;

    while(pipeline.isRunning()) {
        auto result = rigQueue->tryGet<dai::MultiDeviceCalibrationResult>();
        if(result == nullptr) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::cout << result->info << " (data confidence " << result->dataConfidence << ")" << std::endl;
        if(result->passed) {
            dai::CalibrationHandler handler;
            for(const auto& edge : result->calibration.edges) {
                handler.setExtrinsics(edge.from.deviceId, edge.from.socket, edge.to.deviceId, edge.to.socket, edge.transform, edge.transform.lengthUnit);
                const auto transform =
                    handler.getExtrinsics(edge.from.deviceId, edge.from.socket, edge.to.deviceId, edge.to.socket, dai::LengthUnit::CENTIMETER)
                        .getTransformationMatrix();
                std::cout << "  " << dai::toString(edge.from) << " -> " << dai::toString(edge.to) << ": [" << transform[0][3] << ", " << transform[1][3] << ", "
                          << transform[2][3] << "] cm" << std::endl;
            }
            handler.eepromToJsonFile(output);
            std::cout << "Rig written to " << output << std::endl;
        }
        break;
    }

    return 0;
}
