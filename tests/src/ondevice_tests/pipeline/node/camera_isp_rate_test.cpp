#include <catch2/catch_all.hpp>
#include <chrono>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

namespace {

constexpr float WANTED_FPS = 10.0f;

struct RateCheck {
    std::shared_ptr<dai::MessageQueue> reports;
    float expected;
};

RateCheck measure(dai::Pipeline& p, dai::Node::Output* output, float expected) {
    REQUIRE(output != nullptr);
    auto benchmark = p.create<dai::node::BenchmarkIn>();
    benchmark->sendReportEveryNMessages(10);
    output->link(benchmark->input);
    return {benchmark->report.createOutputQueue(4, false), expected};
}

// Skips the first report, which includes the start-up, then requires three reports at the expected rate
void requireRates(const std::vector<RateCheck>& checks) {
    for(const auto& check : checks) {
        CAPTURE(check.expected);
        for(int i = 0; i < 4; ++i) {
            bool timedOut = false;
            const auto report = check.reports->get<dai::BenchmarkReport>(std::chrono::seconds(10), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(report != nullptr);
            if(i > 0) REQUIRE(report->fps == Catch::Approx(check.expected).margin(check.expected * 0.15));
        }
    }
}

}  // namespace

TEST_CASE("An explicit sensor FPS caps a faster ISP output", "[camera-hil]") {
    dai::Pipeline p;
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    const auto& feature = features.front();
    REQUIRE_FALSE(feature.configs.empty());
    const auto& mode = feature.configs.front();
    const float sensorFps = camera_test::fpsWithin(mode, WANTED_FPS);
    CAPTURE(feature.socket, mode.width, mode.height, sensorFps);

    auto camera = p.create<dai::node::Camera>()->build(feature.socket, std::pair(mode.width, mode.height), sensorFps);
    const std::vector<RateCheck> checks{measure(p, camera->requestIspOutput(sensorFps * 3), sensorFps)};
    p.start();
    requireRates(checks);
}

TEST_CASE("Resized outputs and the ISP output share the sensor rate", "[camera-hil]") {
    dai::Pipeline p;
    const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());
    const auto& feature = features.front();
    REQUIRE_FALSE(feature.configs.empty());
    const auto& mode = feature.configs.front();
    const float fps = camera_test::fpsWithin(mode, WANTED_FPS);
    CAPTURE(feature.socket, mode.width, mode.height, fps);

    auto camera = p.create<dai::node::Camera>()->build(feature.socket, std::pair(mode.width, mode.height));
    std::vector<RateCheck> checks;
    for(const auto& size : std::vector<std::pair<uint32_t, uint32_t>>{{640, 400}, {440, 300}, {240, 200}}) {
        checks.push_back(measure(p, camera->requestOutput(size, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, fps), fps));
    }
    // The ISP output does not vote for the sensor rate
    checks.push_back(measure(p, camera->requestIspOutput(), fps));
    p.start();
    requireRates(checks);
}
