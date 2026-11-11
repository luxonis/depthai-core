#pragma once

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

namespace camera_test {

// The first frame of a large sensor mode can take several seconds
constexpr std::chrono::milliseconds FRAME_TIMEOUT{10000};

inline std::shared_ptr<dai::ImgFrame> nextFrame(const std::shared_ptr<dai::MessageQueue>& queue, std::chrono::milliseconds timeout = FRAME_TIMEOUT) {
    bool timedOut = false;
    auto frame = queue->get<dai::ImgFrame>(timeout, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    return frame;
}

inline void drain(const std::shared_ptr<dai::MessageQueue>& queue) {
    while(queue->tryGet() != nullptr) {
    }
}

inline void send(const std::shared_ptr<dai::InputQueue>& queue, const dai::CameraControl& control) {
    queue->send(std::make_shared<dai::CameraControl>(control));
}

// A received frame and a copy of its payload, taken when it arrived
struct HeldFrame {
    std::shared_ptr<dai::ImgFrame> frame;
    std::vector<std::uint8_t> bytes;
};

inline HeldFrame hold(const std::shared_ptr<dai::ImgFrame>& frame) {
    const auto data = frame->getData();
    REQUIRE_FALSE(data.empty());
    return {frame, std::vector<std::uint8_t>(data.begin(), data.end())};
}

// A received frame owns its payload: stopping the pipeline or closing the device must not release or reuse it
inline void requireUnchanged(const std::vector<HeldFrame>& held) {
    for(const auto& entry : held) {
        const auto data = entry.frame->getData();
        REQUIRE(data.size() == entry.bytes.size());
        REQUIRE(std::equal(data.begin(), data.end(), entry.bytes.begin()));
    }
}

inline bool supportsFps(const dai::CameraSensorConfig& config, float fps) {
    return config.minFps <= fps && fps <= config.maxFps;
}

// The rate closest to the wanted one that the sensor config supports
inline float fpsWithin(const dai::CameraSensorConfig& config, float wanted) {
    return std::clamp(wanted, config.minFps, config.maxFps);
}

// The config with the largest area: the sensor's highest resolution
inline const dai::CameraSensorConfig& largestConfig(const dai::CameraFeatures& features) {
    REQUIRE_FALSE(features.configs.empty());
    return *std::max_element(features.configs.begin(), features.configs.end(), [](const auto& a, const auto& b) {
        return static_cast<int64_t>(a.width) * a.height < static_cast<int64_t>(b.width) * b.height;
    });
}

// The features of the camera a built Camera node uses
inline dai::CameraFeatures featuresOf(dai::Pipeline& pipeline, const dai::node::Camera& camera) {
    const auto socket = camera.getBoardSocket();
    for(const auto& features : pipeline.getDefaultDevice()->getConnectedCameraFeatures()) {
        if(features.socket == socket) return features;
    }
    FAIL("No connected camera on the Camera node's socket");
    return {};
}

// The size requestFullResolutionOutput() picks by default: the largest full field of view config up to 5000x4000
inline std::pair<unsigned int, unsigned int> defaultFullResolution(const dai::CameraFeatures& features) {
    std::pair<unsigned int, unsigned int> size{0, 0};
    for(const auto& config : features.configs) {
        const auto fov = config.fov.denormalize(features.width, features.height);
        const bool fullView = fov.x == 0 && fov.y == 0 && static_cast<int>(fov.width) == features.width && static_cast<int>(fov.height) == features.height;
        if(fullView && config.width <= 5000 && config.height <= 4000
           && static_cast<int64_t>(config.width) * config.height > static_cast<int64_t>(size.first) * size.second) {
            size = {config.width, config.height};
        }
    }
    REQUIRE(size.first > 0);
    return size;
}

}  // namespace camera_test
