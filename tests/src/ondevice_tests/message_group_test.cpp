#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/utility/span.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"

bool operator==(const dai::span<uint8_t>& lhs, const std::vector<uint8_t>& rhs) {
    return std::equal(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
}

TEST_CASE("Set and get messages") {
    auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    std::vector<uint8_t> buf1Data = {1, 2, 3};
    std::vector<uint8_t> buf2Data = {4, 5, 6};
    auto msgGrp = std::make_shared<dai::MessageGroup>();
    auto buf = std::make_shared<dai::Buffer>();
    buf->setTimestamp(buf1Ts);
    buf->setData(buf1Data);
    msgGrp->add("buf1", buf);

    auto img = std::make_shared<dai::ImgFrame>();
    img->setTimestamp(buf2Ts);
    img->setData(buf2Data);
    img->setSize({5, 6});
    msgGrp->add("img1", img);

    REQUIRE(msgGrp->get<dai::Buffer>("buf1")->getTimestamp() == buf1Ts);
    REQUIRE(msgGrp->get<dai::ImgFrame>("img1")->getTimestamp() == buf2Ts);
    REQUIRE((msgGrp->get<dai::Buffer>("buf1")->getData() == buf1Data));
    REQUIRE((msgGrp->get<dai::ImgFrame>("img1")->getData() == buf2Data));
    REQUIRE(msgGrp->get<dai::ImgFrame>("img1")->getWidth() == 5);
    REQUIRE(msgGrp->get<dai::ImgFrame>("img1")->getHeight() == 6);
}

TEST_CASE("Send large messages") {
    dai::Pipeline pipeline;
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto sync = pipeline.create<dai::node::Sync>();

    camRgb->requestFullResolutionOutput()->link(sync->inputs["rgb"]);
    left->requestFullResolutionOutput()->link(sync->inputs["left"]);
    right->requestFullResolutionOutput()->link(sync->inputs["right"]);

    auto queue = sync->out.createOutputQueue(8, true);
    pipeline.start();

    bool hasTimedOut = false;
    auto msg = queue->get<dai::MessageGroup>(std::chrono::seconds(20), hasTimedOut);
    // Check that all messages are received
    REQUIRE(!hasTimedOut);
    REQUIRE(msg != nullptr);
    REQUIRE(msg->getNumMessages() == 3);
    for(const auto& name : {"rgb", "left", "right"}) {
        REQUIRE(msg->get(name) != nullptr);
    }
}

// // TODO(asahtik): Does not work on windows[issue](https://github.com/luxonis/depthai-core/issues/929) is fixed
TEST_CASE("Sync - demux") {
    auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

    dai::Pipeline pipeline;
    auto demux = pipeline.create<dai::node::MessageDemux>();
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::milliseconds(100));
    sync->out.link(demux->input);

    auto inQ1 = sync->inputs["msg1"].createInputQueue();
    auto inQ2 = sync->inputs["msg2"].createInputQueue();
    auto outQ1 = demux->outputs["msg1"].createOutputQueue();
    auto outQ2 = demux->outputs["msg2"].createOutputQueue();
    pipeline.start();
    auto buf1 = std::make_shared<dai::Buffer>();
    std::vector<uint8_t> buf1Data = {1, 2, 3, 4, 5};
    std::vector<uint8_t> buf2Data = {6, 7, 8, 9, 10};
    buf1->setData(buf1Data);
    buf1->setTimestamp(buf1Ts);

    auto img1 = std::make_shared<dai::ImgFrame>();
    img1->setData(buf2Data);
    img1->setTimestamp(buf2Ts);
    img1->setSize({5, 6});

    inQ1->send(buf1);
    inQ2->send(img1);
    auto out1 = outQ1->get<dai::Buffer>();
    auto out2 = outQ2->get<dai::ImgFrame>();

    REQUIRE(out1->getTimestamp() == buf1Ts);
    REQUIRE(out2->getTimestamp() == buf2Ts);
    REQUIRE((out1->getData() == buf1Data));
    REQUIRE((out2->getData() == buf2Data));
    REQUIRE(out2->getWidth() == 5);
    REQUIRE(out2->getHeight() == 6);
}

TEST_CASE("MessageGroup ping-pong without XLink") {
    auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

    dai::Pipeline pipeline;
    // Create nodes that are required
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::milliseconds(100));  // Should let the messages through
    auto demux = pipeline.create<dai::node::MessageDemux>();
    sync->out.link(demux->input);

    auto inQ1 = sync->inputs["msg1"].createInputQueue();
    auto inQ2 = sync->inputs["msg2"].createInputQueue();
    auto outQ1 = demux->outputs["msg1"].createOutputQueue();
    auto outQ2 = demux->outputs["msg2"].createOutputQueue();
    pipeline.start();
    // Buffer 1
    auto buf1 = std::make_shared<dai::Buffer>();
    std::vector<uint8_t> buf1Data = {1, 2, 3, 4, 5};
    buf1->setData(buf1Data);
    buf1->setTimestamp(buf1Ts);

    // Image Frame
    auto img1 = std::make_shared<dai::ImgFrame>();
    std::vector<uint8_t> buf2Data = {6, 7, 8, 9, 10};
    img1->setData(buf2Data);
    img1->setTimestamp(buf2Ts);
    img1->setSize({5, 6});

    // Sending the data to queues
    inQ1->send(buf1);
    inQ2->send(img1);

    // Receiving the data back
    auto out1 = outQ1->get<dai::Buffer>();
    auto out2 = outQ2->get<dai::ImgFrame>();

    // Verify buffer data
    REQUIRE(out1 != nullptr);
    REQUIRE(out2 != nullptr);

    // Compare timestamps
    REQUIRE(out1->getTimestamp() == buf1Ts);
    REQUIRE(out2->getTimestamp() == buf2Ts);

    // Compare data using dai::span
    REQUIRE((out1->getData() == buf1Data));
    REQUIRE((out2->getData() == buf2Data));

    // Check image frame size
    REQUIRE(out2->getWidth() == 5);
    REQUIRE(out2->getHeight() == 6);
}