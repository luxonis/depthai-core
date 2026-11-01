#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("Camera: undistorted output uses the GPU, or falls back to the CPU") {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 800;
    constexpr auto kFrameTimeout = std::chrono::seconds(10);
    const std::string kBackendLog = "Undistortion backend: ";

    // Declared before the pipeline, so that it outlives the device and its log callback.
    // Only the log thread writes it. The test reads it after removeLogCallback(), which waits for a running callback.
    std::vector<std::string> backends;

    dai::Pipeline p;
    auto device = p.getDefaultDevice();
    REQUIRE(device->getPlatform() == dai::Platform::RVC4);
    // The camera resizer undistorts on the GPU when there is one, else on the CPU (the HW backend cannot undistort)
    const std::string expectedBackend = device->hasGPU() ? "GPU" : "CPU";
    INFO("Expected undistortion backend: " << expectedBackend);

    // The backend is logged at debug level. Collect it in the callback, but do not print the debug logs
    device->setLogLevel(dai::LogLevel::DEBUG);
    device->setLogOutputLevel(dai::LogLevel::WARN);
    const int callbackId = device->addLogCallback([&](dai::LogMessage message) {
        const auto pos = message.payload.find(kBackendLog);
        if(pos == std::string::npos) return;
        backends.push_back(message.payload.substr(pos + kBackendLog.size()));
    });

    auto camera = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto* output = camera->requestOutput(std::make_pair(kWidth, kHeight), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, 30.0f, true);
    REQUIRE(output != nullptr);
    auto queue = output->createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        bool timedOut = false;
        auto frame = queue->get<dai::ImgFrame>(kFrameTimeout, timedOut);
        if(timedOut) FAIL("Frame timeout: no frame " << i << " from the camera in " << kFrameTimeout.count() << " s");
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getWidth() == kWidth);
        REQUIRE(frame->getHeight() == kHeight);
    }
    p.stop();
    p.wait();
    device->removeLogCallback(callbackId);

    REQUIRE(!backends.empty());
    for(const auto& backend : backends) {
        REQUIRE(backend == expectedBackend);
    }
}
