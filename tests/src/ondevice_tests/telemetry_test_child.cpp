#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "depthai/depthai.hpp"

namespace {

constexpr int kFrameCount = 20;

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}

}  // namespace

int main() {
    try {
        auto device = std::make_shared<dai::Device>();
        const auto cameraFeatures = device->getConnectedCameraFeatures();
        if(cameraFeatures.empty()) {
            fail("No connected cameras were detected");
        }

        const auto selectedCamera = std::find_if(cameraFeatures.begin(), cameraFeatures.end(), [](const dai::CameraFeatures& feature) {
            return feature.socket != dai::CameraBoardSocket::AUTO;
        });
        if(selectedCamera == cameraFeatures.end()) {
            fail("No concrete camera socket was reported by the device");
        }

        dai::Pipeline pipeline(device);
        auto cam = pipeline.create<dai::node::Camera>()->build(selectedCamera->socket);
        auto outputQueue = cam->requestOutput(std::make_pair(640, 400))->createOutputQueue();

        pipeline.start();

        for(int frameIndex = 0; frameIndex < kFrameCount; ++frameIndex) {
            bool timedOut = false;
            auto frame = outputQueue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
            if(timedOut) {
                fail("Timed out waiting for frame " + std::to_string(frameIndex) + " from socket " + dai::toString(selectedCamera->socket));
            }
            if(frame == nullptr) {
                fail("Received a null frame at index " + std::to_string(frameIndex) + " from socket " + dai::toString(selectedCamera->socket));
            }
        }

        outputQueue.reset();
        cam.reset();
        pipeline.stop();
        pipeline.wait();
        return 0;
    } catch(const std::exception& ex) {
        std::cerr << "Telemetry test child failed: " << ex.what() << '\n';
        return 1;
    }
}
