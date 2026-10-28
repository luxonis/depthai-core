#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <map>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("Camera health polling is cumulative and independent across two sensors", "[camera-stats]") {
    using namespace std::chrono_literals;
    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    if(device->getPlatform() != dai::Platform::RVC4) {
        const auto stats = device->getCameraStats();
        for(const auto& sensor : stats) {
            REQUIRE(sensor.status == dai::CameraStats::Status::UNSUPPORTED);
            REQUIRE_FALSE(sensor.completedCaptures);
        }
        return;
    }
    const auto features = device->getConnectedCameraFeatures();
    if(features.size() < 2) SKIP("Two physical RVC4 sensors are required");
    const std::array<dai::CameraBoardSocket, 2> sockets{features[0].socket, features[1].socket};
    REQUIRE(sockets[0] != sockets[1]);
    for(const auto& stats : device->getCameraStats()) {
        REQUIRE(stats.socket != dai::CameraBoardSocket::AUTO);
        REQUIRE(stats.status == dai::CameraStats::Status::INACTIVE);
        REQUIRE_FALSE(stats.completedCaptures);
    }

    std::vector<std::shared_ptr<dai::MessageQueue>> queues;
    for(size_t i = 0; i < sockets.size(); ++i) {
        auto camera = pipeline.create<dai::node::Camera>()->build(sockets[i], std::nullopt, i == 0 ? 30.0f : 15.0f);
        auto* output = camera->requestOutput({320, 240}, std::nullopt, dai::ImgResizeMode::CROP, i == 0 ? 30.0f : 15.0f);
        REQUIRE(output != nullptr);
        queues.push_back(output->createOutputQueue(1, false));
        if(i == 0) {
            auto* second = camera->requestOutput({640, 480}, std::nullopt, dai::ImgResizeMode::CROP, 15.0f);
            REQUIRE(second != nullptr);
            queues.push_back(second->createOutputQueue(1, false));
        }
    }
    pipeline.start();
    std::map<dai::CameraBoardSocket, dai::CameraStats> baseline;
    std::map<dai::CameraBoardSocket, uint64_t> deltas;
    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while(std::chrono::steady_clock::now() < deadline) {
        for(auto& queue : queues) queue->tryGet<dai::ImgFrame>();
        for(const auto& current : device->getCameraStats()) {
            if(current.socket != sockets[0] && current.socket != sockets[1]) continue;
            if(current.status != dai::CameraStats::Status::AVAILABLE) {
                baseline.erase(current.socket);
                deltas.erase(current.socket);
                continue;
            }
            REQUIRE(current.completedCaptures.has_value());
            const auto old = baseline.find(current.socket);
            if(old == baseline.end() || old->second.sessionId != current.sessionId) {
                baseline[current.socket] = current;
                deltas.erase(current.socket);
            } else {
                REQUIRE(current.timestampNs >= old->second.timestampNs);
                REQUIRE(*current.completedCaptures >= *old->second.completedCaptures);
                deltas[current.socket] = *current.completedCaptures - *old->second.completedCaptures;
            }
        }
        if(deltas[sockets[0]] >= 60 && deltas[sockets[1]] >= 25) break;
        std::this_thread::sleep_for(20ms);
    }
    REQUIRE(deltas[sockets[0]] >= 60);
    REQUIRE(deltas[sockets[1]] >= 25);
    // Two outputs on the first sensor must not multiply its successful-capture count.
    const double ratio = static_cast<double>(deltas[sockets[0]]) / deltas[sockets[1]];
    REQUIRE(ratio > 1.4);
    REQUIRE(ratio < 2.7);
    // Stopping Pipeline closes its device transport. A failed RPC must remain an error.
    pipeline.stop();
    REQUIRE_THROWS(device->getCameraStats());
}
