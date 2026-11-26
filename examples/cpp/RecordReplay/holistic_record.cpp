#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

std::filesystem::path recordingsDir() {
    namespace fs = std::filesystem;
    if(const char* xdg = std::getenv("XDG_CACHE_HOME"); xdg != nullptr && xdg[0] != '\0') {
        fs::path path = fs::path(xdg) / "depthai" / "recordings";
        fs::create_directories(path);
        return path;
    }
#if defined(__APPLE__)
    const char* home = std::getenv("HOME");
    fs::path root = (home != nullptr && home[0] != '\0') ? fs::path(home) / "Library" / "Caches" : fs::path();
#elif defined(_WIN32)
    const char* local = std::getenv("LOCALAPPDATA");
    fs::path root;
    if(local != nullptr && local[0] != '\0') {
        root = fs::path(local);
    } else {
        const char* home = std::getenv("USERPROFILE");
        root = (home != nullptr && home[0] != '\0') ? fs::path(home) / "AppData" / "Local" : fs::path();
    }
#else
    const char* home = std::getenv("HOME");
    fs::path root = (home != nullptr && home[0] != '\0') ? fs::path(home) / ".cache" : fs::path();
#endif
    if(root.empty()) {
        throw std::runtime_error("Could not resolve the user cache directory");
    }
    fs::path path = root / "depthai" / "recordings";
    fs::create_directories(path);
    return path;
}

bool containsName(const std::vector<std::string>& names, const std::string& name) {
    for(const auto& item : names) {
        if(item == name) return true;
    }
    return false;
}

bool videoSensor(const dai::CameraFeatures& feature) {
    if(feature.supportedTypes.empty()) return true;
    for(const auto sensorType : feature.supportedTypes) {
        if(sensorType == dai::CameraSensorType::COLOR || sensorType == dai::CameraSensorType::MONO || sensorType == dai::CameraSensorType::AUTO) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main() {
    try {
        dai::Pipeline pipeline;
        auto device = pipeline.getDefaultDevice();
        std::vector<std::shared_ptr<dai::MessageQueue>> queues;
        for(const auto& feature : device->getConnectedCameraFeatures()) {
            if(!videoSensor(feature)) continue;
            auto camera = pipeline.create<dai::node::Camera>()->build(feature.socket);
            auto* frames = camera->requestOutput({640, 480}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.f);
            queues.push_back(frames->createOutputQueue(4, false));
        }
        if(queues.empty()) {
            std::cout << "No cameras found\n";
            return 1;
        }

        const auto imuName = device->getConnectedIMU();
        if(!imuName.empty() && imuName != "NONE") {
            auto imu = pipeline.create<dai::node::IMU>();
            imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
            imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
            imu->setBatchReportThreshold(20);
        }

        const auto output = recordingsDir();
        const bool rvc4 = device->getPlatform() == dai::Platform::RVC4;
        std::vector<std::string> deviceBefore;
        bool haveDeviceList = false;
        if(rvc4) {
            try {
                for(const auto& info : device->listRecordings()) deviceBefore.push_back(info.name);
                haveDeviceList = true;
            } catch(const std::exception&) {
            }
        }

        dai::RecordConfig config;
        config.outputDir = output;
        pipeline.enableHolisticRecord(config);
        std::cout << "Recording to " << output.string() << "\n";
        pipeline.start();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        if(rvc4 && haveDeviceList) {
            const auto lookUntil = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            bool printed = false;
            while(!printed && std::chrono::steady_clock::now() < lookUntil && pipeline.isRunning()) {
                try {
                    for(const auto& info : device->listRecordings()) {
                        if(containsName(deviceBefore, info.name)) continue;
                        std::cout << "/data/depthai/recordings/" << info.name << "\n";
                        printed = true;
                    }
                } catch(const std::exception&) {
                    break;
                }
                if(!printed) std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        while(std::chrono::steady_clock::now() < deadline && pipeline.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        pipeline.stop();
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
    return 0;
}
