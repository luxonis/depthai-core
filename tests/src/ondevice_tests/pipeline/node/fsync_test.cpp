#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

struct Thresholds {
    std::chrono::steady_clock::duration leftRight;
    std::chrono::steady_clock::duration all;
};

constexpr Thresholds RVC2_THRESHOLDS{std::chrono::microseconds(50), std::chrono::milliseconds(5)};
constexpr Thresholds RVC4_THRESHOLDS{std::chrono::microseconds(5), std::chrono::milliseconds(1)};

namespace {

struct CameraRequest {
    dai::CameraBoardSocket socket;
    float fps;
    dai::CameraControl::FrameSyncMode frameSyncMode;
};

struct CameraExpectation {
    dai::CameraBoardSocket socket;
    dai::ImgFrame::Fsync fsync;
    float fps;
};

std::string toString(dai::CameraControl::FrameSyncMode mode) {
    switch(mode) {
        case dai::CameraControl::FrameSyncMode::AUTO:
            return "AUTO";
        case dai::CameraControl::FrameSyncMode::OFF:
            return "OFF";
        case dai::CameraControl::FrameSyncMode::OUTPUT:
            return "OUTPUT";
        case dai::CameraControl::FrameSyncMode::INPUT:
            return "INPUT";
    }
    return "UNKNOWN";
}

std::string toString(dai::ImgFrame::Fsync fsync) {
    switch(fsync) {
        case dai::ImgFrame::Fsync::NONE:
            return "NONE";
        case dai::ImgFrame::Fsync::INPUT:
            return "INPUT";
        case dai::ImgFrame::Fsync::OUTPUT:
            return "OUTPUT";
    }
    return "UNKNOWN";
}

std::shared_ptr<dai::Device> makeRvc4DeviceOrSkip() {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() != dai::Platform::RVC4) {
        SKIP("These metadata fsync tests are intended for RVC4 only");
    }
    return device;
}

std::map<dai::CameraBoardSocket, std::shared_ptr<dai::ImgFrame>> collectOneFramePerCamera(const std::vector<CameraRequest>& requests,
                                                                                          const std::vector<dai::CameraBoardSocket>& expectedSlaveSockets,
                                                                                          const std::vector<dai::StereoPair>& stereoPairs,
                                                                                          std::shared_ptr<dai::Device> device) {
    dai::Pipeline pipeline(std::move(device));
    std::map<dai::CameraBoardSocket, std::shared_ptr<dai::MessageQueue>> queues;
    std::shared_ptr<dai::node::Sync> sync = nullptr;
    std::shared_ptr<dai::MessageQueue> syncQueue = nullptr;
    if(expectedSlaveSockets.size() >= 2) {
        sync = pipeline.create<dai::node::Sync>();
        std::optional<float> expectedSlaveFps;
        for(const auto& request : requests) {
            if(std::find(expectedSlaveSockets.begin(), expectedSlaveSockets.end(), request.socket) != expectedSlaveSockets.end()) {
                expectedSlaveFps = request.fps;
                break;
            }
        }
        REQUIRE(expectedSlaveFps.has_value());
        const auto syncThreshold = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / expectedSlaveFps.value()));
        sync->setSyncThreshold(syncThreshold);
    }

    for(const auto& request : requests) {
        auto camera = pipeline.create<dai::node::Camera>()->build(request.socket);
        camera->initialControl.setFrameSyncMode(request.frameSyncMode);
        const auto outputSize = request.socket == dai::CameraBoardSocket::CAM_A ? std::make_pair(1280U, 800U) : std::make_pair(640U, 400U);
        auto* output = camera->requestOutput(outputSize, std::nullopt, dai::ImgResizeMode::CROP, request.fps);
        REQUIRE(output != nullptr);
        queues.emplace(request.socket, output->createOutputQueue(4, false));
        if(sync != nullptr && std::find(expectedSlaveSockets.begin(), expectedSlaveSockets.end(), request.socket) != expectedSlaveSockets.end()) {
            output->link(sync->inputs[dai::toString(request.socket)]);
        }
    }

    if(sync != nullptr) {
        syncQueue = sync->out.createOutputQueue(4, false);
    }
    pipeline.start();

    if(syncQueue != nullptr) {
        for(int i = 0; i < 5; ++i) {
            auto syncedFrames = syncQueue->get<dai::MessageGroup>();
            REQUIRE(syncedFrames != nullptr);
        }

        for(int i = 0; i < 20; ++i) {
            auto syncedFrames = syncQueue->get<dai::MessageGroup>();
            REQUIRE(syncedFrames != nullptr);
            std::vector<std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>> slaveTimestamps;
            for(const auto socket : expectedSlaveSockets) {
                auto frame = syncedFrames->get<dai::ImgFrame>(dai::toString(socket));
                REQUIRE(frame != nullptr);
                slaveTimestamps.push_back(frame->getTimestampDevice());
            }

            const auto [minIt, maxIt] = std::minmax_element(slaveTimestamps.begin(), slaveTimestamps.end());
            auto threshold = RVC4_THRESHOLDS.all;
            const bool expectedPairIsStereo = std::find_if(stereoPairs.begin(),
                                                           stereoPairs.end(),
                                                           [&](const auto& pair) {
                                                               return (pair.left == expectedSlaveSockets[0] && pair.right == expectedSlaveSockets[1])
                                                                      || (pair.left == expectedSlaveSockets[1] && pair.right == expectedSlaveSockets[0]);
                                                           })
                                              != stereoPairs.end();
            if(expectedSlaveSockets.size() == 2 && expectedPairIsStereo) {
                threshold = RVC4_THRESHOLDS.leftRight;
            }
            REQUIRE((*maxIt - *minIt) <= threshold);
        }
    }

    std::map<dai::CameraBoardSocket, std::shared_ptr<dai::ImgFrame>> frames;
    for(const auto& request : requests) {
        auto frame = queues.at(request.socket)->get<dai::ImgFrame>();
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getFps() == Catch::Approx(request.fps).margin(0.05));
        REQUIRE(frame->getSensorMode() >= 0);
        frames.emplace(request.socket, frame);
    }

    return frames;
}

void requireExpectedMetadata(const std::map<dai::CameraBoardSocket, std::shared_ptr<dai::ImgFrame>>& frames,
                             const std::vector<CameraExpectation>& expectations) {
    REQUIRE(frames.size() == expectations.size());
    for(const auto& expectation : expectations) {
        const auto it = frames.find(expectation.socket);
        REQUIRE(it != frames.end());
        CAPTURE(dai::toString(expectation.socket), toString(it->second->getFsync()), toString(expectation.fsync));
        REQUIRE(it->second->getFsync() == expectation.fsync);
        REQUIRE(it->second->getFps() == Catch::Approx(expectation.fps).margin(0.05));
        REQUIRE(it->second->getSensorMode() >= 0);
    }
}

bool isPartOfStereoPair(dai::CameraBoardSocket socket, const std::vector<dai::StereoPair>& stereoPairs) {
    return std::find_if(stereoPairs.begin(), stereoPairs.end(), [socket](const auto& pair) { return pair.left == socket || pair.right == socket; })
           != stereoPairs.end();
}

void requireExactlyOneStereoPairedSlave(const std::map<dai::CameraBoardSocket, std::shared_ptr<dai::ImgFrame>>& frames,
                                        const std::vector<dai::StereoPair>& stereoPairs,
                                        const std::vector<CameraRequest>& requests) {
    std::vector<dai::CameraBoardSocket> slaves;
    for(const auto& request : requests) {
        const auto frame = frames.at(request.socket);
        REQUIRE(frame->getFps() == Catch::Approx(request.fps).margin(0.05));
        REQUIRE(frame->getSensorMode() >= 0);
        if(frame->getFsync() == dai::ImgFrame::Fsync::INPUT) {
            slaves.push_back(request.socket);
        } else {
            REQUIRE(frame->getFsync() == dai::ImgFrame::Fsync::NONE);
        }
    }

    REQUIRE(slaves.size() == 1);
    REQUIRE(isPartOfStereoPair(slaves.front(), stereoPairs));
}

void requireStartThrows(const std::vector<CameraRequest>& requests, const std::string& errorFragment) {
    auto device = makeRvc4DeviceOrSkip();
    dai::Pipeline pipeline(std::move(device));

    for(const auto& request : requests) {
        auto camera = pipeline.create<dai::node::Camera>()->build(request.socket);
        camera->initialControl.setFrameSyncMode(request.frameSyncMode);
        auto* output = camera->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, request.fps);
        REQUIRE(output != nullptr);
        output->createOutputQueue(4, false);
    }

    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring(errorFragment));
}

}  // namespace

void testFsync(float fps, Thresholds thresholds, std::shared_ptr<dai::Device> device) {
    // Create pipeline
    dai::Pipeline p(std::move(device));
    auto rgb = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto sync = p.create<dai::node::Sync>();
    // Convert frame sync threshold to nanoseconds
    auto thresholdNs = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / fps));

    sync->setSyncThreshold(thresholdNs);
    left->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["left"]);
    right->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["right"]);
    rgb->requestOutput(std::make_pair(1280, 800), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(sync->inputs["rgb"]);

    auto benchmarkIn = p.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(25);
    sync->out.link(benchmarkIn->input);
    auto benchmarkQueue = benchmarkIn->report.createOutputQueue(10, false);

    auto syncQueue = sync->out.createOutputQueue(10, false);
    p.start();

    for(int i = 0; i < 100; i++) {
        auto syncData = syncQueue->get<dai::MessageGroup>();
        REQUIRE(syncData != nullptr);

        auto leftFrame = syncData->get<dai::ImgFrame>("left");
        REQUIRE(leftFrame != nullptr);
        auto rightFrame = syncData->get<dai::ImgFrame>("right");
        REQUIRE(rightFrame != nullptr);
        auto rgbFrame = syncData->get<dai::ImgFrame>("rgb");
        REQUIRE(rgbFrame != nullptr);

        auto rgbTimestamp = rgbFrame->getTimestampDevice();
        auto leftTimestamp = leftFrame->getTimestampDevice();
        auto rightTimestamp = rightFrame->getTimestampDevice();

        // Compute the absolute difference between left and right timestamps.
        auto diffLeftRight = (leftTimestamp > rightTimestamp) ? (leftTimestamp - rightTimestamp) : (rightTimestamp - leftTimestamp);

        // Compute the difference between the maximum and minimum timestamps among all frames.
        auto maxTimestamp = std::max({leftTimestamp, rightTimestamp, rgbTimestamp});
        auto minTimestamp = std::min({leftTimestamp, rightTimestamp, rgbTimestamp});
        auto diffAll = maxTimestamp - minTimestamp;

        // Verify that the differences are within the expected thresholds.
        // 'thresholds.leftRight' applies to the left/right pair, while
        // 'thresholds.all' applies to the overall synchronization among all frames.
        REQUIRE(diffLeftRight <= thresholds.leftRight);
        REQUIRE(diffAll <= thresholds.all);
    }

    for(int i = 0; i < 4; i++) {
        auto reportData = benchmarkQueue->get<dai::BenchmarkReport>();
        if(i < 2) {
            // Skip the first two reports, to let the FPS stabilize.
            continue;
        }
        REQUIRE(reportData != nullptr);
        REQUIRE(reportData->numMessagesReceived > 1);
        if(!p.isHolisticReplayEnabled()) REQUIRE(reportData->fps == Catch::Approx(fps).epsilon(0.1));
    }
}

void testFsyncIntegrated(float fps) {
    // Create device
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() == dai::Platform::RVC4) {
        testFsync(fps, RVC4_THRESHOLDS, device);
    } else if(device->getPlatform() == dai::Platform::RVC2) {
        testFsync(fps, RVC2_THRESHOLDS, device);
    } else {
        throw std::runtime_error("Unsupported platform");
    }
}

TEST_CASE("Test Fsync with different FPS values", "[fsync]") {
    // Specify a list of FPS values to test with.
    auto fps = GENERATE(10.0f, 13.0f, 18.5f, 30.0f);
    CAPTURE(fps);
    testFsyncIntegrated(fps);
}

TEST_CASE("Fsync metadata state selection", "[fsync][metadata]") {
    auto device = makeRvc4DeviceOrSkip();
    const auto stereoPairs = device->getStereoPairs();

    SECTION("A-A@30 | B-A@30 | C-A@30") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(
            requests, {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::INPUT, 30.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::INPUT, 30.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::INPUT, 30.0f},
                                });
    }

    SECTION("A-A@30 | B-A@55 | C-A@23") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 55.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 23.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {}, stereoPairs, device);
        requireExactlyOneStereoPairedSlave(frames, stereoPairs, requests);
    }

    SECTION("A-A@30 | B-A@20 | C-A@20") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::NONE, 30.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                });
    }

    SECTION("A-A@20 | B-A@20 | C-A@30") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_B}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::NONE, 30.0f},
                                });
    }

    SECTION("A-A@20 | B-A@30 | C-A@20") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 20.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::NONE, 30.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::INPUT, 20.0f},
                                });
    }

    SECTION("A-S@23 | B-A@30 | C-A@30") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 23.0f, dai::CameraControl::FrameSyncMode::INPUT},
            {dai::CameraBoardSocket::CAM_B, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::INPUT, 23.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::NONE, 30.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::NONE, 30.0f},
                                });
    }

    SECTION("A-A@30 | B-A@23 | C-S@23") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 23.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_C, 23.0f, dai::CameraControl::FrameSyncMode::INPUT},
        };
        auto frames = collectOneFramePerCamera(requests, {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::NONE, 30.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::INPUT, 23.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::INPUT, 23.0f},
                                });
    }

    SECTION("A-A@30 | B-N@30 | C-A@30") {
        const std::vector<CameraRequest> requests = {
            {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
            {dai::CameraBoardSocket::CAM_B, 30.0f, dai::CameraControl::FrameSyncMode::OFF},
            {dai::CameraBoardSocket::CAM_C, 30.0f, dai::CameraControl::FrameSyncMode::AUTO},
        };
        auto frames = collectOneFramePerCamera(requests, {dai::CameraBoardSocket::CAM_A, dai::CameraBoardSocket::CAM_C}, stereoPairs, device);
        requireExpectedMetadata(frames,
                                {
                                    {dai::CameraBoardSocket::CAM_A, dai::ImgFrame::Fsync::INPUT, 30.0f},
                                    {dai::CameraBoardSocket::CAM_B, dai::ImgFrame::Fsync::NONE, 30.0f},
                                    {dai::CameraBoardSocket::CAM_C, dai::ImgFrame::Fsync::INPUT, 30.0f},
                                });
    }
}

TEST_CASE("Fsync invalid configurations error out", "[fsync][metadata]") {
    SECTION("A-S@30 | B-S@55") {
        requireStartThrows(
            {
                {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::INPUT},
                {dai::CameraBoardSocket::CAM_B, 55.0f, dai::CameraControl::FrameSyncMode::INPUT},
            },
            "only 1 fsync fps");
    }

    SECTION("A-M@30") {
        requireStartThrows(
            {
                {dai::CameraBoardSocket::CAM_A, 30.0f, dai::CameraControl::FrameSyncMode::OUTPUT},
            },
            "unsupported on RVC4");
    }
}
