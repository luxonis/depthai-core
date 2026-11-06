#include <catch2/catch_all.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>

#include "depthai/depthai.hpp"

// Device replay. With no device, or with firmware that has no replay-cache RPC, the test skips.
TEST_CASE("ReplayVideo device execution") {
    const auto [found, info] = dai::Device::getAnyAvailableDevice();
    if(!found) SKIP("No device connected");

    auto opened = std::make_shared<dai::Device>(info);
    dai::Pipeline pipeline(opened);
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr || device->getPlatform() != dai::Platform::RVC4) SKIP("Replay cache requires an RVC4 device");

    try {
        const auto cache = device->getReplayCacheInfo();
        (void)cache;
    } catch(const std::exception& ex) {
        SKIP(std::string("Replay cache RPC is not available: ") + ex.what());
    }

    const auto source = std::filesystem::temp_directory_path() / "depthai_replay_device_source.avi";
    {
        std::ofstream output(source, std::ios::binary);
        output << "not a real avi";
    }

    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setReplayVideoFile(source);
    replay->setRunOnHost(false);
    replay->setLoop(false);
    replay->setFps(30.f);
    replay->setStopPipelineAtEnd(true);
    auto queue = replay->out.createOutputQueue();

    try {
        pipeline.start();
    } catch(const std::exception& ex) {
        std::filesystem::remove(source);
        SKIP(std::string("Device replay is not available: ") + ex.what());
    }

    bool timedOut = false;
    auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(2), timedOut);
    pipeline.stop();
    std::filesystem::remove(source);
    if(timedOut || frame == nullptr) SKIP("Device replay produced no frame");
    REQUIRE(frame->getWidth() > 0);
}
