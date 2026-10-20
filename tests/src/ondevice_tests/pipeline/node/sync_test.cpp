#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <optional>
#include <thread>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Camera.hpp"

namespace {

using SteadyTimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;
using SystemTimePoint = std::chrono::time_point<std::chrono::system_clock, std::chrono::system_clock::duration>;

SteadyTimePoint steadyMs(int ms) {
    return SteadyTimePoint{std::chrono::milliseconds(ms)};
}

SystemTimePoint systemMs(int ms) {
    return SystemTimePoint{std::chrono::milliseconds(ms)};
}

struct TestBufferTimestamps {
    int64_t sequenceNum;
    SteadyTimePoint hostTimestamp;
    SteadyTimePoint deviceTimestamp;
    std::optional<SystemTimePoint> systemTimestamp;
};

TestBufferTimestamps makeTimestamps(int64_t sequenceNum,
                                    SteadyTimePoint hostTimestamp,
                                    SteadyTimePoint deviceTimestamp,
                                    std::optional<SystemTimePoint> systemTimestamp) {
    return TestBufferTimestamps{sequenceNum, hostTimestamp, deviceTimestamp, systemTimestamp};
}

std::shared_ptr<dai::Buffer> makeBuffer(const TestBufferTimestamps& timestamps) {
    auto buffer = std::make_shared<dai::Buffer>();
    buffer->setSequenceNum(timestamps.sequenceNum);
    buffer->setTimestamp(timestamps.hostTimestamp);
    buffer->setTimestampDevice(timestamps.deviceTimestamp);
    buffer->setTimestampSystem(timestamps.systemTimestamp);
    buffer->setData({static_cast<uint8_t>(timestamps.sequenceNum & 0xFF)});
    return buffer;
}

void test_camera_sync(bool runSyncOnHost) {
    // Create pipeline
    dai::Pipeline p;
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto sync = p.create<dai::node::Sync>();
    sync->setRunOnHost(runSyncOnHost);
    left->requestFullResolutionOutput()->link(sync->inputs["left"]);
    right->requestFullResolutionOutput()->link(sync->inputs["right"]);

    auto syncQueue = sync->out.createOutputQueue();
    p.start();

    for(int i = 0; i < 10; i++) {
        auto syncData = syncQueue->get<dai::MessageGroup>();
        REQUIRE(syncData != nullptr);

        auto leftFrame = syncData->get<dai::ImgFrame>("left");
        REQUIRE(leftFrame != nullptr);
        auto rightFrame = syncData->get<dai::ImgFrame>("right");
        REQUIRE(rightFrame != nullptr);

        auto groupTimestamp = syncData->getTimestamp();
        auto leftTimestamp = leftFrame->getTimestamp();
        auto rightTimestamp = rightFrame->getTimestamp();
        REQUIRE(groupTimestamp > std::chrono::steady_clock::time_point{});
        REQUIRE(groupTimestamp >= leftTimestamp);
        REQUIRE(groupTimestamp >= rightTimestamp);
    }
}

void test_sync_timestamp_source(dai::node::Sync::TimestampSource source,
                                const TestBufferTimestamps& leftTimestamps,
                                const TestBufferTimestamps& rightTimestamps) {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setTimestampSource(source);
    sync->setSyncThreshold(std::chrono::milliseconds(10));

    auto leftInputQueue = sync->inputs["left"].createInputQueue();
    auto rightInputQueue = sync->inputs["right"].createInputQueue();
    auto syncQueue = sync->out.createOutputQueue(4, false);

    pipeline.start();

    leftInputQueue->send(makeBuffer(leftTimestamps));
    rightInputQueue->send(makeBuffer(rightTimestamps));

    bool hasTimedOut = false;
    auto syncData = syncQueue->get<dai::MessageGroup>(std::chrono::seconds(1), hasTimedOut);

    REQUIRE_FALSE(hasTimedOut);
    REQUIRE(syncData != nullptr);

    auto leftBuffer = syncData->get<dai::Buffer>("left");
    auto rightBuffer = syncData->get<dai::Buffer>("right");
    REQUIRE(leftBuffer != nullptr);
    REQUIRE(rightBuffer != nullptr);
    REQUIRE(leftBuffer->getSequenceNum() == leftTimestamps.sequenceNum);
    REQUIRE(rightBuffer->getSequenceNum() == rightTimestamps.sequenceNum);

    // The output group inherits all metadata from the newest frame according to the selected timestamp source.
    REQUIRE(syncData->getSequenceNum() == rightTimestamps.sequenceNum);
    REQUIRE(syncData->getTimestamp() == rightTimestamps.hostTimestamp);
    REQUIRE(syncData->getTimestampDevice() == rightTimestamps.deviceTimestamp);
    REQUIRE(syncData->getTimestampSystem() == rightTimestamps.systemTimestamp);
}

void waitUntilPipelineStops(const dai::Pipeline& pipeline, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(pipeline.isRunning() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}  // namespace

TEST_CASE("Sync node runs on device") {
    test_camera_sync(false);
}

TEST_CASE("Sync node runs on host") {
    test_camera_sync(true);
}

TEST_CASE("Sync node uses host timestamps when selected") {
    test_sync_timestamp_source(dai::node::Sync::TimestampSource::HOST,
                               makeTimestamps(1, steadyMs(100), steadyMs(400), systemMs(700)),
                               makeTimestamps(2, steadyMs(105), steadyMs(100), systemMs(200)));
}

TEST_CASE("Sync node uses device timestamps when selected") {
    test_sync_timestamp_source(dai::node::Sync::TimestampSource::DEVICE,
                               makeTimestamps(1, steadyMs(400), steadyMs(100), systemMs(700)),
                               makeTimestamps(2, steadyMs(100), steadyMs(105), systemMs(200)));
}

TEST_CASE("Sync node uses system timestamps when selected") {
    test_sync_timestamp_source(dai::node::Sync::TimestampSource::SYSTEM,
                               makeTimestamps(1, steadyMs(400), steadyMs(700), systemMs(100)),
                               makeTimestamps(2, steadyMs(100), steadyMs(200), systemMs(105)));
}

TEST_CASE("Sync node stops when system timestamps are selected and a message is missing one") {
    dai::Pipeline pipeline(false);
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setTimestampSource(dai::node::Sync::TimestampSource::SYSTEM);
    sync->setSyncThreshold(std::chrono::milliseconds(10));

    auto leftInputQueue = sync->inputs["left"].createInputQueue();
    auto rightInputQueue = sync->inputs["right"].createInputQueue();
    auto syncQueue = sync->out.createOutputQueue(4, false);

    pipeline.start();

    leftInputQueue->send(makeBuffer(makeTimestamps(1, steadyMs(100), steadyMs(100), systemMs(100))));
    rightInputQueue->send(makeBuffer(makeTimestamps(2, steadyMs(105), steadyMs(105), std::nullopt)));

    waitUntilPipelineStops(pipeline, std::chrono::seconds(1));

    REQUIRE_FALSE(pipeline.isRunning());
    REQUIRE_THROWS_AS(syncQueue->get<dai::MessageGroup>(), dai::MessageQueue::QueueException);
}
