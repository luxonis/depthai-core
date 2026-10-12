#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

constexpr int kFrameCount = 20;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}

struct DeviceScenario {
    dai::DeviceInfo deviceInfo;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::node::Camera> cam;
    std::shared_ptr<dai::MessageQueue> outputQueue;
    dai::Pipeline pipeline;
    std::string queueKey;
    std::string socketName;
    int framesReceived = 0;

    explicit DeviceScenario(const dai::DeviceInfo& info) : deviceInfo(info), device(std::make_shared<dai::Device>(info)), pipeline(device) {}
};

void stopScenarios(std::vector<DeviceScenario>& scenarios) {
    for(auto& scenario : scenarios) {
        scenario.outputQueue.reset();
        scenario.cam.reset();
        scenario.pipeline.stop();
    }
    for(auto& scenario : scenarios) {
        scenario.pipeline.wait();
    }
}

}  // namespace

int main() {
    try {
        auto availableDevices = dai::Device::getAllAvailableDevices();
        if(availableDevices.empty()) {
            fail("No available devices were detected");
        }

        std::sort(availableDevices.begin(), availableDevices.end(), [](const dai::DeviceInfo& lhs, const dai::DeviceInfo& rhs) {
            return lhs.getDeviceId() < rhs.getDeviceId();
        });

        std::vector<DeviceScenario> scenarios;
        scenarios.reserve(availableDevices.size());
        for(const auto& deviceInfo : availableDevices) {
            scenarios.emplace_back(deviceInfo);
            auto& scenario = scenarios.back();

            const auto cameraFeatures = scenario.device->getConnectedCameraFeatures();
            if(cameraFeatures.empty()) {
                fail("No connected cameras were detected on device " + deviceInfo.toString());
            }

            const auto selectedCamera = std::find_if(cameraFeatures.begin(), cameraFeatures.end(), [](const dai::CameraFeatures& feature) {
                return feature.socket != dai::CameraBoardSocket::AUTO;
            });
            if(selectedCamera == cameraFeatures.end()) {
                fail("No concrete camera socket was reported by device " + deviceInfo.toString());
            }

            scenario.cam = scenario.pipeline.create<dai::node::Camera>()->build(selectedCamera->socket);
            scenario.outputQueue = scenario.cam->requestOutput(std::make_pair(640, 400))->createOutputQueue();
            scenario.queueKey = scenario.device->getDeviceId();
            scenario.socketName = dai::toString(selectedCamera->socket);
        }

        for(auto& scenario : scenarios) {
            scenario.pipeline.start();
        }

        while(std::any_of(scenarios.begin(), scenarios.end(), [](const DeviceScenario& scenario) { return scenario.framesReceived < kFrameCount; })) {
            std::vector<std::reference_wrapper<dai::MessageQueue>> activeQueues;
            for(auto& scenario : scenarios) {
                if(scenario.framesReceived < kFrameCount) {
                    activeQueues.emplace_back(std::ref(*scenario.outputQueue));
                }
            }

            bool hadReadyFrame = false;
            const auto waitStartedAt = std::chrono::steady_clock::now();
            while(!hadReadyFrame && std::chrono::steady_clock::now() - waitStartedAt < std::chrono::seconds(10)) {
                dai::MessageQueue::waitAny(activeQueues);
                for(auto& scenario : scenarios) {
                    if(scenario.framesReceived < kFrameCount && scenario.outputQueue->has()) {
                        hadReadyFrame = true;
                        break;
                    }
                }
            }

            if(!hadReadyFrame) {
                std::string message = "Timed out waiting for frames from devices:";
                for(const auto& scenario : scenarios) {
                    if(scenario.framesReceived < kFrameCount) {
                        message += " " + scenario.deviceInfo.toString() + " (" + std::to_string(scenario.framesReceived) + "/" + std::to_string(kFrameCount)
                                   + ")";
                    }
                }
                fail(message);
            }

            for(auto& scenario : scenarios) {
                if(scenario.framesReceived >= kFrameCount || !scenario.outputQueue->has()) {
                    continue;
                }
                auto frame = scenario.outputQueue->get<dai::ImgFrame>();
                if(frame == nullptr) {
                    fail("Received a null frame from socket " + scenario.socketName + " on device " + scenario.deviceInfo.toString());
                }
                scenario.framesReceived += 1;
            }
        }

        stopScenarios(scenarios);
        return 0;
    } catch(const std::exception& ex) {
        std::cerr << "Telemetry multi-device test child failed: " << ex.what() << '\n';
        return 1;
    }
}
