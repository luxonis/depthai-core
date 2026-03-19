#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <type_traits>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

using namespace std::chrono_literals;

namespace {

struct FrameSpec {
    int64_t sequenceNum;
    std::chrono::time_point<std::chrono::steady_clock> steadyTs;
    std::chrono::time_point<std::chrono::system_clock> systemTs;
    std::chrono::time_point<dai::ptp_clock> ptpTs;
};

std::shared_ptr<dai::ImgFrame> createFrame(const FrameSpec& spec) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setSequenceNum(spec.sequenceNum);
    frame->setTimestamp(spec.steadyTs);
    frame->setTimestampDevice(spec.steadyTs);
    #ifndef DEPTHAI_MESSAGES_RVC2
    frame->setTimestampSystem(spec.systemTs);
    frame->setTimestampPtp(spec.ptpTs);
    #endif
    frame->setData(std::vector<std::uint8_t>{static_cast<uint8_t>(spec.sequenceNum)});
    return frame;
}

template<typename SyncNode>
void fillTimestampScenario(FrameSpec& leftFirst, FrameSpec& leftSecond, FrameSpec& right) {
    if constexpr(std::is_same_v<SyncNode, dai::node::Sync>) {
        leftFirst = {1, std::chrono::steady_clock::time_point{100ns}, std::chrono::system_clock::time_point{1100ns}, std::chrono::time_point<dai::ptp_clock>{2100ns}};
        leftSecond = {2, std::chrono::steady_clock::time_point{200ns}, std::chrono::system_clock::time_point{1200ns}, std::chrono::time_point<dai::ptp_clock>{2200ns}};
        right = {3, std::chrono::steady_clock::time_point{203ns}, std::chrono::system_clock::time_point{1300ns}, std::chrono::time_point<dai::ptp_clock>{2300ns}};
    } else if constexpr(std::is_same_v<SyncNode, dai::node::SyncSystem>) {
        leftFirst = {1, std::chrono::steady_clock::time_point{100ns}, std::chrono::system_clock::time_point{100ns}, std::chrono::time_point<dai::ptp_clock>{2100ns}};
        leftSecond = {2, std::chrono::steady_clock::time_point{300ns}, std::chrono::system_clock::time_point{200ns}, std::chrono::time_point<dai::ptp_clock>{2200ns}};
        right = {3, std::chrono::steady_clock::time_point{200ns}, std::chrono::system_clock::time_point{203ns}, std::chrono::time_point<dai::ptp_clock>{2300ns}};
    } else {
        static_assert(std::is_same_v<SyncNode, dai::node::SyncPtp>, "Unsupported sync node type");
        leftFirst = {1, std::chrono::steady_clock::time_point{100ns}, std::chrono::system_clock::time_point{1100ns}, std::chrono::time_point<dai::ptp_clock>{100ns}};
        leftSecond = {2, std::chrono::steady_clock::time_point{300ns}, std::chrono::system_clock::time_point{1200ns}, std::chrono::time_point<dai::ptp_clock>{200ns}};
        right = {3, std::chrono::steady_clock::time_point{200ns}, std::chrono::system_clock::time_point{1300ns}, std::chrono::time_point<dai::ptp_clock>{203ns}};
    }
}

void testCameraSync(bool runSyncOnHost) {
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

template<typename SyncNode>
void testTimestampSelection(bool runSyncOnHost) {
    dai::Pipeline pipeline;
    auto sync = pipeline.create<SyncNode>();
    sync->setRunOnHost(runSyncOnHost);
    sync->setSyncThreshold(5ns);

    auto leftQueue = sync->inputs["left"].createInputQueue();
    auto rightQueue = sync->inputs["right"].createInputQueue();
    auto outputQueue = sync->out.createOutputQueue();

    FrameSpec leftFirst;
    FrameSpec leftSecond;
    FrameSpec right;
    fillTimestampScenario<SyncNode>(leftFirst, leftSecond, right);

    pipeline.start();

    leftQueue->send(createFrame(leftFirst));
    rightQueue->send(createFrame(right));
    leftQueue->send(createFrame(leftSecond));

    bool hasTimedOut = false;
    auto syncData = outputQueue->template get<dai::MessageGroup>(5s, hasTimedOut);

    REQUIRE_FALSE(hasTimedOut);
    REQUIRE(syncData != nullptr);
    REQUIRE(syncData->getNumMessages() == 2);

    auto leftFrame = syncData->template get<dai::ImgFrame>("left");
    auto rightFrame = syncData->template get<dai::ImgFrame>("right");
    REQUIRE(leftFrame != nullptr);
    REQUIRE(rightFrame != nullptr);

    REQUIRE(leftFrame->getSequenceNum() == leftSecond.sequenceNum);
    REQUIRE(rightFrame->getSequenceNum() == right.sequenceNum);
    REQUIRE(leftFrame->getTimestamp() == leftSecond.steadyTs);
    REQUIRE(rightFrame->getTimestamp() == right.steadyTs);

    #ifndef DEPTHAI_MESSAGES_RVC2
    auto leftSystemTs = leftFrame->getTimestampSystem();
    auto rightSystemTs = rightFrame->getTimestampSystem();
    REQUIRE(leftSystemTs.has_value());
    REQUIRE(rightSystemTs.has_value());
    REQUIRE(*leftSystemTs == leftSecond.systemTs);
    REQUIRE(*rightSystemTs == right.systemTs);

    auto leftPtpTs = leftFrame->getTimestampPtp();
    auto rightPtpTs = rightFrame->getTimestampPtp();
    REQUIRE(leftPtpTs.has_value());
    REQUIRE(rightPtpTs.has_value());
    REQUIRE(*leftPtpTs == leftSecond.ptpTs);
    REQUIRE(*rightPtpTs == right.ptpTs);

    REQUIRE(syncData->getSequenceNum() == right.sequenceNum);
    REQUIRE(syncData->getTimestamp() == right.steadyTs);
    REQUIRE(syncData->getTimestampDevice() == right.steadyTs);
    REQUIRE(syncData->getTimestampSystem().has_value());
    REQUIRE(syncData->getTimestampPtp().has_value());
    REQUIRE(*syncData->getTimestampSystem() == right.systemTs);
    REQUIRE(*syncData->getTimestampPtp() == right.ptpTs);
    #endif
}

}  // namespace

TEST_CASE("Sync node groups camera frames on device") {
    testCameraSync(false);
}

TEST_CASE("Sync node groups camera frames on host") {
    testCameraSync(true);
}

TEST_CASE("Sync node uses steady timestamps on device") {
    testTimestampSelection<dai::node::Sync>(false);
}

TEST_CASE("Sync node uses steady timestamps on host") {
    testTimestampSelection<dai::node::Sync>(true);
}

#ifndef DEPTHAI_MESSAGES_RVC2
TEST_CASE("SyncSystem node uses system timestamps on device") {
    // testTimestampSelection<dai::node::SyncSystem>(false);
    // TODO: Re-enable once depthai-device exposes/registers the "SyncSystem" gate node.
    // Currently the device-side PipelineBuilder only knows about "Sync", so this
    // test fails at runtime with: "Pipeline node with name: 'SyncSystem' doesn't exist".
    SKIP("SyncSystem is not registered on the device side yet");
}

TEST_CASE("SyncSystem node uses system timestamps on host") {
    testTimestampSelection<dai::node::SyncSystem>(true);
}

TEST_CASE("SyncPtp node uses PTP timestamps on device") {
    // testTimestampSelection<dai::node::SyncPtp>(false);
    // TODO: Re-enable once depthai-device exposes/registers the "SyncPtp" gate node.
    // Currently the device-side PipelineBuilder only knows about "Sync", so this
    // test fails at runtime with: "Pipeline node with name: 'SyncPtp' doesn't exist".
    SKIP("SyncPtp is not registered on the device side yet");
}

TEST_CASE("SyncPtp node uses PTP timestamps on host") {
    testTimestampSelection<dai::node::SyncPtp>(true);
}
#endif
