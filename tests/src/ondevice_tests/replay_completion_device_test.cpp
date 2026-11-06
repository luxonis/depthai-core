#include <catch2/catch_all.hpp>
#include <chrono>
#include <future>
#include <iostream>
#include <thread>

#include "../ReplayArchiveFixture.hpp"
#include "depthai/depthai.hpp"
#include "depthai/schemas/ImgFrame.pb.h"
#include "utility/RecordReplayImpl.hpp"

using namespace std::chrono_literals;

namespace {

std::shared_ptr<dai::Device> openRvc4() {
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(info.platform == X_LINK_RVC4) return std::make_shared<dai::Device>(info);
    }
    return nullptr;
}

std::filesystem::path writeSource(const std::filesystem::path& root, bool metadata, int count) {
    const auto frames = root / "frames";
    std::filesystem::create_directory(frames);
    const auto mcap = root / "frames.mcap";
    dai::utility::ByteRecorder recorder;
    if(metadata) recorder.init<dai::proto::img_frame::ImgFrame>(mcap.string(), dai::RecordConfig::CompressionLevel::NONE, "video");
    for(int index = 0; index < count; ++index) {
        dai::ImgFrame frame;
        frame.setType(dai::ImgFrame::Type::NV12);
        frame.setSize(64, 48);
        frame.setSequenceNum(index);
        const auto time = std::chrono::steady_clock::time_point(std::chrono::milliseconds(index * 100));
        frame.setTimestamp(time);
        frame.setTimestampDevice(time);
        frame.setData(std::vector<std::uint8_t>(64 * 48 * 3 / 2, 100));
        if(metadata) recorder.write(frame.serializeProto());
        else frame.save(frames / ("cam_" + std::to_string(index) + ".dai"));
    }
    if(metadata) recorder.close();
    return metadata ? mcap : frames;
}

std::shared_ptr<dai::MessageQueue> addReplay(dai::Pipeline& pipeline,
                                              const std::filesystem::path& source,
                                              bool metadata,
                                              bool loop = false,
                                              bool stopAtEnd = true) {
    if(metadata) {
        auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
        replay->setReplayFile(source);
        replay->setRunOnHost(false).setLoop(loop).setStopPipelineAtEnd(stopAtEnd).setFps(10);
        return replay->out.createOutputQueue(32, false);
    }
    auto replay = pipeline.create<dai::node::ReplayVideo>();
    replay->setSource(source).setRunOnHost(false).setLoop(loop).setStopPipelineAtEnd(stopAtEnd).setFps(10);
    return replay->out.createOutputQueue(32, false);
}

void requireCompleted(dai::Pipeline& pipeline, const std::shared_ptr<dai::Device>& device, std::future<void>& execution,
                      std::chrono::steady_clock::time_point deadline) {
    const bool completed = execution.wait_until(deadline) == std::future_status::ready;
    // Release a blocked wait even when testing the original broken implementation.
    if(!completed) pipeline.stop();
    execution.get();
    REQUIRE(completed);
    REQUIRE_FALSE(pipeline.isRunning());
    REQUIRE(device->isClosed());
}

class IdleHostNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, IdleHostNode> {
   public:
    constexpr static const char* NAME = "ReplayCompletionIdleHost";
    void run() override {
        while(mainLoop()) std::this_thread::sleep_for(5ms);
    }
};

}  // namespace

TEST_CASE("Device replay completion stops the host and releases wait", "[replay][completion][rvc4]") {
    const bool metadata = GENERATE(false, true);
    const int count = GENERATE(1, 5);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory files;
    const auto source = writeSource(files.path, metadata, count);
    dai::Pipeline pipeline(device);
    auto queue = addReplay(pipeline, source, metadata);
    pipeline.create<IdleHostNode>();
    pipeline.start();
    const auto start = std::chrono::steady_clock::now();
    auto completion = std::async(std::launch::async, [&] { pipeline.wait(); });
    requireCompleted(pipeline, device, completion, start + (count - 1) * 100ms + 1s);
    std::cout << "metadata=" << metadata << " frames=" << count << " waitMs="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() << '\n';
}

TEST_CASE("Device replay completion releases run", "[replay][completion][rvc4]") {
    const bool metadata = GENERATE(false, true);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory files;
    dai::Pipeline pipeline(device);
    auto queue = addReplay(pipeline, writeSource(files.path, metadata, 5), metadata);
    auto completion = std::async(std::launch::async, [&] { pipeline.run(); });
    // Include device pipeline startup in this deadline, separate from the LP-3 wait test.
    requireCompleted(pipeline, device, completion, std::chrono::steady_clock::now() + 5s);
}

TEST_CASE("Device replay honors loop and stop-at-end policies", "[replay][completion][rvc4]") {
    const bool metadata = GENERATE(false, true);
    const bool loop = GENERATE(false, true);
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory files;
    dai::Pipeline pipeline(device);
    auto queue = addReplay(pipeline, writeSource(files.path, metadata, 3), metadata, loop, loop);
    pipeline.start();
    for(int index = 0; index < (loop ? 7 : 3); ++index) {
        bool timeout = false;
        const auto frame = queue->get<dai::ImgFrame>(3s, timeout);
        REQUIRE_FALSE(timeout);
        REQUIRE(frame);
    }
    std::this_thread::sleep_for(300ms);
    REQUIRE(pipeline.isRunning());
    REQUIRE(device->isPipelineRunning());
    pipeline.stop();
    pipeline.wait();
    REQUIRE_FALSE(pipeline.isRunning());
}

TEST_CASE("The first device replay to finish stops looping siblings", "[replay][completion][rvc4]") {
    auto device = openRvc4();
    if(!device) SKIP("No RVC4 connected");
    replaytest::Directory files;
    dai::Pipeline pipeline(device);
    const auto source = writeSource(files.path, false, 1);
    auto looping = addReplay(pipeline, source, false, true, true);
    auto finite = addReplay(pipeline, source, false);
    pipeline.start();
    auto completion = std::async(std::launch::async, [&] { pipeline.wait(); });
    requireCompleted(pipeline, device, completion, std::chrono::steady_clock::now() + 1s);
}
