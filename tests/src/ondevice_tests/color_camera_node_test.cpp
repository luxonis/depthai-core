#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

TEST_CASE("Test ColorCamera node") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto colorCam = p.create<dai::node::ColorCamera>();

    int previewWidth, previewHeight;
    SECTION("640x320") {
        previewWidth = 640, previewHeight = 320;
        colorCam->setPreviewSize(previewWidth, previewHeight);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(true);
    }

    SECTION("1280x720") {
        previewWidth = 1280, previewHeight = 720;
        colorCam->setPreviewSize(previewWidth, previewHeight);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
        colorCam->setInterleaved(false);
    }

    auto previewQueue = colorCam->preview.createOutputQueue();
    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto preview = previewQueue->get<dai::ImgFrame>();

        // Check output dimensions
        REQUIRE(preview->getWidth() == previewWidth);
        REQUIRE(preview->getHeight() == previewHeight);
    }
}
