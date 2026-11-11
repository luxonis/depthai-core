#include <catch2/catch_all.hpp>
#include <utility>
#include <vector>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

using Catch::Matchers::ContainsSubstring;

TEST_CASE("Camera outputs take the largest feasible size of a range or a list", "[.][capability-selection]") {
    dai::Pipeline p;
    auto camera = p.create<dai::node::Camera>()->build();
    const auto maxWidth = camera->getMaxWidth();
    const auto maxHeight = camera->getMaxHeight();
    REQUIRE(maxWidth >= 640);
    REQUIRE(maxHeight >= 480);

    dai::ImgFrameCapability ranged;
    ranged.size.minMax(std::pair<uint32_t, uint32_t>{320, 240}, std::pair<uint32_t, uint32_t>{640, 480});
    ranged.fps.minMax(10.0f, 120.0f);
    ranged.type = dai::ImgFrame::Type::GRAY8;

    // The larger listed size exceeds the sensor unless the sensor is at least 8192x8192
    dai::ImgFrameCapability discrete;
    discrete.size.discrete({{640, 480}, {8192, 8192}});
    discrete.fps.discrete({10.0f, 120.0f});
    discrete.type = dai::ImgFrame::Type::GRAY8;
    const bool fitsLarger = maxWidth >= 8192 && maxHeight >= 8192;
    const auto discreteSize = fitsLarger ? std::pair<unsigned int, unsigned int>{8192, 8192} : std::pair<unsigned int, unsigned int>{640, 480};

    const std::vector<std::pair<std::shared_ptr<dai::MessageQueue>, std::pair<unsigned int, unsigned int>>> outputs{
        {camera->requestOutput(ranged, false)->createOutputQueue(2, false), {640, 480}},
        {camera->requestOutput(discrete, false)->createOutputQueue(2, false), discreteSize},
    };
    p.start();
    for(const auto& [queue, size] : outputs) {
        for(int i = 0; i < 3; ++i) {
            const auto frame = camera_test::nextFrame(queue);
            REQUIRE(frame->getWidth() == size.first);
            REQUIRE(frame->getHeight() == size.second);
            REQUIRE(frame->getType() == dai::ImgFrame::Type::GRAY8);
        }
    }
}

TEST_CASE("Camera refuses to start an output it cannot produce", "[camera-hil]") {
    dai::Pipeline p;
    const auto platform = p.getDefaultDevice()->getPlatform();
    auto camera = p.create<dai::node::Camera>()->build();
    // One pixel wide: no even crop of a sensor mode has this aspect ratio, and the width is odd
    camera->requestOutput({1, 1000}, dai::ImgFrame::Type::GRAY8)->createOutputQueue();
    if(platform == dai::Platform::RVC2NG) {
        REQUIRE_THROWS_WITH(p.start(), ContainsSubstring("Camera output aspect ratio requires an empty aligned crop"));
    } else if(platform == dai::Platform::RVC4) {
        REQUIRE_THROWS_WITH(p.start(), ContainsSubstring("We only support even numbers for resolution sizes. Got 1x1000."));
    } else {
        FAIL("No expected rejection for platform " << dai::platform2string(platform));
    }
}

TEST_CASE("Opaque NATIVE output streams only in the normal orientation", "[.][native-orientation]") {
    SECTION("The normal orientation streams capture-sized NATIVE frames") {
        dai::Pipeline p;
        REQUIRE(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG);
        auto camera = p.create<dai::node::Camera>()->build();
        const auto size = camera_test::defaultFullResolution(camera_test::featuresOf(p, *camera));
        auto queue = camera->requestFullResolutionOutput(dai::ImgFrame::Type::NATIVE)->createOutputQueue(1, false);
        p.start();
        const auto frame = camera_test::nextFrame(queue);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NATIVE);
        REQUIRE(frame->getWidth() == size.first);
        REQUIRE(frame->getHeight() == size.second);
    }
    SECTION("A mirrored NATIVE output is refused") {
        dai::Pipeline p;
        REQUIRE(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG);
        auto camera = p.create<dai::node::Camera>()->build();
        camera->setImageOrientation(dai::CameraImageOrientation::HORIZONTAL_MIRROR);
        camera->requestFullResolutionOutput(dai::ImgFrame::Type::NATIVE)->createOutputQueue(1, false);
        REQUIRE_THROWS_WITH(p.start(), ContainsSubstring("Opaque NATIVE output does not support image orientation"));
    }
}
