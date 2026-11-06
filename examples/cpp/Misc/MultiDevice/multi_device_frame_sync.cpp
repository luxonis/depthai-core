
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

std::atomic_bool running{true};

void interruptHandler(int) {
    running = false;
}

struct ParsedArgs {
    float targetFps = 30.0f;
    // Unset: 1 ms with hardware sync, 20 ms with host sync only
    std::optional<float> syncThresholdSec;
    bool externalSync = false;
    bool ptpSync = false;
    std::vector<std::string> deviceArgs;
};

std::optional<ParsedArgs> parseArguments(int argc, char** argv) {
    ParsedArgs parsed;
    auto printUsage = [argv]() {
        std::cout << "Usage: " << argv[0]
                  << " [-f|--fps <target_fps>] [-d|--devices <device_1> [device_2 ...]]"
                     " [-t|--sync-threshold-sec <sec>] [--external-sync|--ptp-sync]"
                  << std::endl;
    };
    try {
        for(int i = 1; i < argc;) {
            const std::string arg = argv[i];
            if(arg == "-f" || arg == "--fps") {
                if(i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
                parsed.targetFps = std::stof(argv[i + 1]);
                i += 2;
            } else if(arg == "-t" || arg == "--sync-threshold-sec") {
                if(i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
                parsed.syncThresholdSec = std::stof(argv[i + 1]);
                i += 2;
            } else if(arg == "-d" || arg == "--devices") {
                i++;
                while(i < argc && argv[i][0] != '-') {
                    parsed.deviceArgs.emplace_back(argv[i++]);
                }
                if(parsed.deviceArgs.empty()) throw std::runtime_error("Option " + arg + " requires at least one device");
            } else if(arg == "--external-sync") {
                parsed.externalSync = true;
                i++;
            } else if(arg == "--ptp-sync") {
                parsed.ptpSync = true;
                i++;
            } else {
                throw std::runtime_error("Unknown option: " + arg);
            }
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        printUsage();
        return std::nullopt;
    }
    if(parsed.externalSync && parsed.ptpSync) {
        std::cerr << "Specify at most one of --external-sync or --ptp-sync" << std::endl;
        printUsage();
        return std::nullopt;
    }
    return parsed;
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGINT, interruptHandler);

    auto parsed = parseArguments(argc, argv);
    if(!parsed.has_value()) return 1;

    const bool hostSyncOnly = !parsed->externalSync && !parsed->ptpSync;
    const float syncThresholdSec = parsed->syncThresholdSec.value_or(hostSyncOnly ? 20e-3f : 1e-3f);

    std::vector<dai::DeviceInfo> deviceInfos;
    if(parsed->deviceArgs.empty()) {
        deviceInfos = dai::Device::getAllAvailableDevices();
    } else {
        for(const auto& arg : parsed->deviceArgs) deviceInfos.emplace_back(arg);
    }
    if(deviceInfos.size() < 2) {
        std::cout << "At least two devices are required for this example." << std::endl;
        return 0;
    }

    dai::Pipeline pipeline(false);

    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(std::chrono::nanoseconds(static_cast<int64_t>(std::round(1e9 * syncThresholdSec))));

    std::vector<std::string> inputNames;
    for(auto& info : deviceInfos) {
        auto device = pipeline.addDevice(info);
        if(device->getPlatform() != dai::Platform::RVC4) {
            throw std::runtime_error("This example supports only the RVC4 platform!");
        }

        std::optional<dai::ExternalFrameSyncRole> role;
        if(parsed->externalSync) {
            role = device->getExternalFrameSyncRole();
            if(role == dai::ExternalFrameSyncRole::MASTER) {
                device->setExternalStrobeEnable(true);
            }
        }

        std::shared_ptr<dai::node::Camera> cam;
        if(!parsed->externalSync || role == dai::ExternalFrameSyncRole::MASTER) {
            cam = pipeline.create<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A, std::nullopt, parsed->targetFps);
        } else {
            // FSYNC slaves lock to the master's strobe
            cam = pipeline.create<dai::node::Camera>(device)->build(dai::CameraBoardSocket::CAM_A, std::nullopt);
        }
        if(parsed->ptpSync) {
            cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::TIME_PTP);
        }
        auto name = device->getDeviceId();
        cam->requestOutput(std::make_pair(1280, 800), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::STRETCH)->link(sync->inputs[name]);
        inputNames.push_back(name);
    }

    auto queue = sync->out.createOutputQueue();
    pipeline.start();

    while(running && pipeline.isRunning()) {
        bool hasTimedOut = false;
        auto group = queue->get<dai::MessageGroup>(std::chrono::milliseconds(500), hasTimedOut);
        if(group == nullptr) continue;

        // Newest minus oldest frame timestamp of the group
        const double maxDiffMs = static_cast<double>(group->getIntervalNs()) / 1e6;

        std::vector<cv::Mat> frames;
        for(const auto& name : inputNames) {
            auto frame = group->get<dai::ImgFrame>(name);
            if(frame == nullptr) continue;
            auto img = frame->getCvFrame();
            std::ostringstream label;
            label << name << "  " << std::fixed << std::setprecision(3) << std::chrono::duration<double>(frame->getTimestamp().time_since_epoch()).count()
                  << " s";
            cv::putText(img, label.str(), {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 127, 255}, 2, cv::LINE_AA);
            frames.push_back(img);
        }
        if(!frames.empty()) {
            cv::Mat combined;
            cv::hconcat(frames, combined);
            std::ostringstream diff;
            diff << "max diff = " << std::fixed << std::setprecision(2) << maxDiffMs << " ms";
            cv::putText(combined, diff.str(), {20, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2, cv::LINE_AA);
            cv::imshow("multi_device_frame_sync", combined);
            if(cv::waitKey(1) == 'q') break;
        }
    }

    pipeline.stop();
    return 0;
}
