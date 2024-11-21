#include <catch2/catch_all.hpp>

// Include depthai library
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/MessageDemux.hpp"

bool operator==(const dai::span<uint8_t>& lhs, const dai::span<uint8_t>& rhs) {
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

void testBridgeMessages(const std::vector<size_t>& sizes) {
    dai::Pipeline p;
    // Use sync node to force a minimal bridge to the device
    auto sync = p.create<dai::node::Sync>();
    auto demux = p.create<dai::node::MessageDemux>();
    sync->out.link(demux->input);
    // Create the input and output queues
    auto inputQueue = sync->inputs["first"].createInputQueue();
    auto outputQueue = demux->outputs["first"].createOutputQueue();
    // Start the pipeline
    p.start();
    for(auto size : sizes) {
        // Send a message
        auto message = std::make_shared<dai::Buffer>();
        message->setData(std::vector<std::uint8_t>(size, 3));
        inputQueue->send(message);
        auto receivedMessage = outputQueue->get<dai::Buffer>();
        REQUIRE(receivedMessage != nullptr);
        REQUIRE(receivedMessage->getData().size() == size);
        auto equal = (receivedMessage->getData() == message->getData());
        REQUIRE(equal);
    }
}

TEST_CASE("Bridge large messages") {
    testBridgeMessages({0, 1, 100, 100000, 200000, 300000, 100 * 1024 * 1024, 100 * 1024 * 1024 + 1});
}
