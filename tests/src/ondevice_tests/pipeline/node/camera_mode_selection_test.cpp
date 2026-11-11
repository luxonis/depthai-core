#include <algorithm>
#include <catch2/catch_all.hpp>
#include <optional>
#include <string>
#include <utility>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

using camera_test::fpsWithin;
using camera_test::largestConfig;
using camera_test::nextFrame;

TEST_CASE("The highest-resolution full output uses the sensor's largest size", "[camera-hil]") {
    dai::Pipeline p;
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    const auto& feature = features.front();
    const float fps = fpsWithin(largestConfig(feature), 10.0f);
    CAPTURE(feature.socket, feature.width, feature.height, fps);

    auto camera = p.create<dai::node::Camera>()->build(feature.socket);
    REQUIRE(camera->getMaxWidth() == static_cast<uint32_t>(feature.width));
    REQUIRE(camera->getMaxHeight() == static_cast<uint32_t>(feature.height));
    auto queue = camera->requestFullResolutionOutput(dai::ImgFrame::Type::GRAY8, fps, true)->createOutputQueue(1, false);
    p.start();
    const auto frame = nextFrame(queue);
    REQUIRE(frame->getWidth() == camera->getMaxWidth());
    REQUIRE(frame->getHeight() == camera->getMaxHeight());
    REQUIRE(frame->getType() == dai::ImgFrame::Type::GRAY8);
}

// The ISP output has the size of the sensor mode the Camera selects
TEST_CASE("The ISP output follows the selected sensor mode", "[.][isp-mode]") {
    enum class Scenario { SMALL_OUTPUT, ISP_ONLY, EXPLICIT_RESOLUTION };
    const auto scenario = GENERATE(Scenario::SMALL_OUTPUT, Scenario::ISP_ONLY, Scenario::EXPLICIT_RESOLUTION);
    CAPTURE(static_cast<int>(scenario));

    dai::Pipeline p;
    REQUIRE(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG);
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    const auto& feature = features.front();
    const auto& largest = largestConfig(feature);
    CAPTURE(feature.socket, feature.width, feature.height);

    std::pair<unsigned int, unsigned int> expected;
    std::shared_ptr<dai::MessageQueue> queue;
    if(scenario == Scenario::SMALL_OUTPUT) {
        // A small output at a rate the sensor supports selects the smallest mode with that rate
        const bool has30 = std::any_of(feature.configs.begin(), feature.configs.end(), [](const auto& c) { return camera_test::supportsFps(c, 30.0f); });
        const float fps = has30 ? 30.0f : feature.configs.front().maxFps;
        const dai::CameraSensorConfig* smallest = nullptr;
        for(const auto& config : feature.configs) {
            if(camera_test::supportsFps(config, fps) && (!smallest || config.width * config.height < smallest->width * smallest->height)) smallest = &config;
        }
        REQUIRE(smallest != nullptr);
        expected = {smallest->width, smallest->height};
        auto camera = p.create<dai::node::Camera>()->build(feature.socket);
        camera->requestOutput({200, 200}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps)->createOutputQueue(1, false);
        queue = camera->requestIspOutput(std::min(10.0f, fps))->createOutputQueue(1, false);
    } else if(scenario == Scenario::ISP_ONLY) {
        // An ISP output alone selects the largest mode
        expected = {largest.width, largest.height};
        auto camera = p.create<dai::node::Camera>()->build(feature.socket);
        queue = camera->requestIspOutput(fpsWithin(largest, 10.0f))->createOutputQueue(1, false);
    } else {
        // An explicit sensor resolution wins over a small output
        expected = {largest.width, largest.height};
        const float fps = fpsWithin(largest, 30.0f);
        auto camera = p.create<dai::node::Camera>()->build(feature.socket, std::pair(largest.width, largest.height));
        camera->requestOutput({200, 200}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps)->createOutputQueue(1, false);
        queue = camera->requestIspOutput(std::min(10.0f, fps))->createOutputQueue(1, false);
    }
    p.start();
    const auto frame = nextFrame(queue);
    REQUIRE(frame->getWidth() == expected.first);
    REQUIRE(frame->getHeight() == expected.second);
}
