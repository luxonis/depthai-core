#include <catch2/catch_test_macros.hpp>

#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/remote_connection/RemoteConnection.hpp"
#define CATCH_CONFIG_MAIN
#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

TEST_CASE("Basic remote connection test") {
    dai::RemoteConnection remoteConnection;
    auto inputQueue = remoteConnection.addTopic("input", "group");
}

TEST_CASE("Basic remote connection test with topic removal") {
    dai::RemoteConnection remoteConnection;
    for(int i = 0; i < 10; i++) {
        auto imgFrame = std::make_shared<dai::ImgFrame>();
        auto inputQueue = remoteConnection.addTopic("input", "group");
        inputQueue->send(imgFrame);
        remoteConnection.removeTopic("input");
        REQUIRE_THROWS_AS(inputQueue->send(imgFrame), dai::MessageQueue::QueueException);
    }
}
