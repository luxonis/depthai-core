#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

TEST_CASE("Test ColorCamera node") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    DEPTHAI_BEGIN_SUPPRESS_DEPRECATION_WARNING
    auto colorCam = p.create<dai::node::ColorCamera>();
    DEPTHAI_END_SUPPRESS_DEPRECATION_WARNING

    int previewWidth = 0;
    int previewHeight = 0;
    SECTION("640x320") {
        previewWidth = 640, previewHeight = 320;
        colorCam->setPreviewSize(previewWidth, previewHeight);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setPreviewType(dai::ImgFrame::Type::BGR888i);
    }

    SECTION("1280x720") {
        previewWidth = 1280, previewHeight = 720;
        colorCam->setPreviewSize(previewWidth, previewHeight);
        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
        colorCam->setPreviewType(dai::ImgFrame::Type::BGR888p);
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
