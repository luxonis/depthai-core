#include <catch2/catch_all.hpp>
#include <optional>
#include <vector>

#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

using Catch::Matchers::ContainsSubstring;

TEST_CASE("Camera ISP pool defaults follow the platform and reach the device", "[camera-hil]") {
    const auto count = GENERATE(std::optional<int>{}, std::optional<int>{3});
    CAPTURE(count.value_or(-1));

    dai::Pipeline p;
    const bool rvc2ng = p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG;
    // RVC2NG keeps four capture frames with no byte limit; other platforms keep three frames within 10 MiB
    const int expectedCount = count.value_or(rvc2ng ? 4 : 3);
    const int expectedBytes = rvc2ng ? -1 : 10 * 1024 * 1024;

    auto camera = p.create<dai::node::Camera>()->build();
    if(count) camera->setIspNumFramesPool(*count);
    REQUIRE(camera->getIspNumFramesPool() == expectedCount);
    REQUIRE(camera->getMaxSizePoolIsp() == expectedBytes);

    const auto serialized = p.serializeToJson(false);
    std::vector<nlohmann::json> properties;
    for(const auto& node : serialized["pipeline"]["nodes"]) {
        if(node[1]["name"] == "Camera") properties.push_back(node[1]["properties"]);
    }
    REQUIRE(properties.size() == 1);
    REQUIRE(properties.front()["numFramesPoolIsp"] == expectedCount);
    REQUIRE(properties.front()["maxSizePoolIsp"] == expectedBytes);

    auto queue = camera->requestOutput({320, 240}, dai::ImgFrame::Type::GRAY8)->createOutputQueue(1, false);
    p.start();
    camera_test::nextFrame(queue);
    REQUIRE(camera->getIspNumFramesPool() == expectedCount);
    REQUIRE(camera->getMaxSizePoolIsp() == expectedBytes);
}

TEST_CASE("Automatic pools deliver the full-resolution output, and its frame outlives the device", "[camera-hil]") {
    std::vector<camera_test::HeldFrame> held;
    {
        dai::Pipeline p;
        auto camera = p.create<dai::node::Camera>()->build();
        const auto size = camera_test::defaultFullResolution(camera_test::featuresOf(p, *camera));
        CAPTURE(size.first, size.second);
        auto queue = camera->requestFullResolutionOutput()->createOutputQueue(1, false);
        p.start();
        const auto frame = camera_test::nextFrame(queue);
        REQUIRE(frame->getWidth() == size.first);
        REQUIRE(frame->getHeight() == size.second);
        REQUIRE(frame->getData().size() >= static_cast<size_t>(size.first) * size.second);
        held.push_back(camera_test::hold(frame));
        p.stop();
        p.wait();
        camera_test::requireUnchanged(held);
    }
    camera_test::requireUnchanged(held);
}

TEST_CASE("An ISP byte limit below one frame is refused", "[.][isp-byte-limit]") {
    dai::Pipeline p;
    REQUIRE(p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG);
    auto camera = p.create<dai::node::Camera>()->build();
    camera->setMaxSizePoolIsp(1);
    camera->requestFullResolutionOutput()->createOutputQueue(1, false);
    REQUIRE_THROWS_WITH(p.start(), ContainsSubstring("Camera frame exceeds the configured pool size limit"));
}
