#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"

void test_sync(bool runSyncOnHost) {
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

TEST_CASE("Sync node runs on device") {
    test_sync(false);
}

TEST_CASE("Sync node runs on host") {
    test_sync(true);
}