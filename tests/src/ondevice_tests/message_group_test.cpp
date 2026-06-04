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

namespace {

std::shared_ptr<dai::Buffer> makeBuffer(uint8_t value) {
    auto buffer = std::make_shared<dai::Buffer>();
    buffer->setData(std::vector<uint8_t>{value});
    return buffer;
}

std::shared_ptr<dai::MessageGroup> makeTreeMessageGroup() {
    auto group = std::make_shared<dai::MessageGroup>();
    for(uint32_t nodeIndex = 0; nodeIndex <= 14; ++nodeIndex) {
        group->add(nodeIndex, makeBuffer(static_cast<uint8_t>(10 + nodeIndex)));
    }

    REQUIRE(group->addLink(0, 1, 0) == 0);
    REQUIRE(group->addLink(0, 2, 1) == 1);
    REQUIRE(group->addLink(0, 3, 2) == 2);
    REQUIRE(group->addLink(1, 4, 0) == 3);
    REQUIRE(group->addLink(1, 5, 1) == 4);
    REQUIRE(group->addLink(2, 6, 0) == 5);
    REQUIRE(group->addLink(2, 7, 1) == 6);
    REQUIRE(group->addLink(3, 8, 0) == 7);
    REQUIRE(group->addLink(3, 9, 1) == 8);
    REQUIRE(group->addLink(4, 10, 0) == 9);
    REQUIRE(group->addLink(4, 11, 1) == 10);
    REQUIRE(group->addLink(5, 12, 0) == 11);
    REQUIRE(group->addLink(dai::Link{7, 0, 13}) == 12);
    REQUIRE(group->addLink(dai::Link{9, 0, 14}) == 13);

    return group;
}

}  // namespace

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

TEST_CASE("MessageGroup tree structure helpers") {
    SECTION("Query tree structure") {
        auto msgGrp = makeTreeMessageGroup();

        auto replacement = makeBuffer(99);
        REQUIRE(msgGrp->getNode(0) != nullptr);
        REQUIRE(msgGrp->getNode(14) != nullptr);
        REQUIRE(msgGrp->getNode(15) == nullptr);
        REQUIRE(msgGrp->setNode(4, replacement));
        REQUIRE(msgGrp->getNode(4) == replacement);
        REQUIRE_FALSE(msgGrp->setNode(99, replacement));

        REQUIRE(msgGrp->hasLink(0, 0, 1));
        REQUIRE(msgGrp->hasLink(0, 1, 2));
        REQUIRE(msgGrp->hasLink(0, 2, 3));
        REQUIRE(msgGrp->hasLink(1, 0, 4));
        REQUIRE(msgGrp->hasLink(1, 1, 5));
        REQUIRE(msgGrp->hasLink(2, 0, 6));
        REQUIRE(msgGrp->hasLink(2, 1, 7));
        REQUIRE(msgGrp->hasLink(3, 0, 8));
        REQUIRE(msgGrp->hasLink(3, 1, 9));
        REQUIRE(msgGrp->hasLink(4, 0, 10));
        REQUIRE(msgGrp->hasLink(4, 1, 11));
        REQUIRE(msgGrp->hasLink(5, 0, 12));
        REQUIRE(msgGrp->hasLink(7, 0, 13));
        REQUIRE(msgGrp->hasLink(9, 0, 14));
        REQUIRE_FALSE(msgGrp->hasLink(6, 0, 14));

        auto linksFromRoot = msgGrp->getLinksFromParent(0);
        REQUIRE(linksFromRoot.size() == 3);
        REQUIRE(linksFromRoot[0].itemIndex == 0);
        REQUIRE(linksFromRoot[0].childNodeIndex == 1);
        REQUIRE(linksFromRoot[1].itemIndex == 1);
        REQUIRE(linksFromRoot[1].childNodeIndex == 2);
        REQUIRE(linksFromRoot[2].itemIndex == 2);
        REQUIRE(linksFromRoot[2].childNodeIndex == 3);

        auto linksFromNode1Item1 = msgGrp->getLinksFromParent(1, 1);
        REQUIRE(linksFromNode1Item1.size() == 1);
        REQUIRE(linksFromNode1Item1[0].childNodeIndex == 5);

        auto linksToNode14 = msgGrp->getLinksToChild(14);
        REQUIRE(linksToNode14.size() == 1);
        REQUIRE(linksToNode14[0].parentMessageIndex == 9);
        REQUIRE(linksToNode14[0].itemIndex == 0);

        REQUIRE(msgGrp->getChildren(0) == std::vector<int>{1, 2, 3});
        REQUIRE(msgGrp->getChildren(1) == std::vector<int>{4, 5});
        REQUIRE(msgGrp->getChildren(2) == std::vector<int>{6, 7});
        REQUIRE(msgGrp->getChildren(3) == std::vector<int>{8, 9});
        REQUIRE(msgGrp->getChildren(4) == std::vector<int>{10, 11});
        REQUIRE(msgGrp->getChildren(5) == std::vector<int>{12});
        REQUIRE(msgGrp->getChildren(7) == std::vector<int>{13});
        REQUIRE(msgGrp->getChildren(9) == std::vector<int>{14});
        REQUIRE(msgGrp->getChildren(0, 0) == std::vector<int>{1});
        REQUIRE(msgGrp->getChildren(0, 1) == std::vector<int>{2});
        REQUIRE(msgGrp->getChildren(0, 2) == std::vector<int>{3});
        REQUIRE(msgGrp->getParents(1) == std::vector<int>{0});
        REQUIRE(msgGrp->getParents(14) == std::vector<int>{9});

        REQUIRE(msgGrp->isRoot(0));
        REQUIRE_FALSE(msgGrp->isRoot(14));
        REQUIRE_FALSE(msgGrp->isRoot(1));
        REQUIRE_FALSE(msgGrp->isRoot(9));

        REQUIRE(msgGrp->isLeaf(6));
        REQUIRE(msgGrp->isLeaf(8));
        REQUIRE(msgGrp->isLeaf(10));
        REQUIRE(msgGrp->isLeaf(11));
        REQUIRE(msgGrp->isLeaf(12));
        REQUIRE(msgGrp->isLeaf(13));
        REQUIRE(msgGrp->isLeaf(14));
        REQUIRE_FALSE(msgGrp->isLeaf(2));
        REQUIRE_FALSE(msgGrp->isLeaf(4));
        REQUIRE_FALSE(msgGrp->isLeaf(0));
        REQUIRE_FALSE(msgGrp->isLeaf(1));
        REQUIRE_FALSE(msgGrp->isLeaf(3));
        REQUIRE_FALSE(msgGrp->isLeaf(5));
        REQUIRE_FALSE(msgGrp->isLeaf(7));
        REQUIRE_FALSE(msgGrp->isLeaf(9));

        REQUIRE(msgGrp->getRootMessageNodes() == std::vector<int>{0});
        REQUIRE(msgGrp->depthFirstOrder(0) == std::vector<int>{0, 1, 4, 10, 11, 5, 12, 2, 6, 7, 13, 3, 8, 9, 14});
        REQUIRE(msgGrp->breadthFirstOrder(0) == std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14});
    }

    SECTION("Mutate tree structure") {
        auto msgGrp = makeTreeMessageGroup();

        REQUIRE(msgGrp->removeLink(4));
        REQUIRE_FALSE(msgGrp->removeLink(99));
        REQUIRE_FALSE(msgGrp->hasLink(1, 1, 5));
        REQUIRE(msgGrp->getChildren(1) == std::vector<int>{4});

        REQUIRE(msgGrp->removeMessageNode(7));
        REQUIRE_FALSE(msgGrp->removeMessageNode(99));
        REQUIRE(msgGrp->getNode(7) != nullptr);
        REQUIRE(msgGrp->getNode(13) != nullptr);
        REQUIRE(msgGrp->getNode(14) == nullptr);
        REQUIRE_FALSE(msgGrp->hasLink(2, 1, 7));
        REQUIRE(msgGrp->hasLink(3, 0, 7));
        REQUIRE(msgGrp->hasLink(3, 1, 8));
        REQUIRE(msgGrp->hasLink(4, 0, 9));
        REQUIRE(msgGrp->hasLink(4, 1, 10));
        REQUIRE(msgGrp->hasLink(5, 0, 11));
        REQUIRE(msgGrp->hasLink(8, 0, 13));
        REQUIRE(msgGrp->getChildren(3) == std::vector<int>{7, 8});
        REQUIRE(msgGrp->getChildren(8) == std::vector<int>{13});
        REQUIRE(msgGrp->getRootMessageNodes() == std::vector<int>{0, 5, 12});
        REQUIRE(msgGrp->depthFirstOrder(0) == std::vector<int>{0, 1, 4, 9, 10, 2, 6, 3, 7, 8, 13});
        REQUIRE(msgGrp->breadthFirstOrder(0) == std::vector<int>{0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 13});
    }
}

// TEST_CASE("Send large messages") {
//     dai::Pipeline pipeline;
//     auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
//     auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
//     auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

//     auto sync = pipeline.create<dai::node::Sync>();

//     camRgb->requestFullResolutionOutput()->link(sync->inputs["rgb"]);
//     left->requestFullResolutionOutput()->link(sync->inputs["left"]);
//     right->requestFullResolutionOutput()->link(sync->inputs["right"]);

//     auto queue = sync->out.createOutputQueue(8, true);
//     pipeline.start();

//     bool hasTimedOut = false;
//     auto msg = queue->get<dai::MessageGroup>(std::chrono::seconds(20), hasTimedOut);
//     // Check that all messages are received
//     REQUIRE(!hasTimedOut);
//     REQUIRE(msg != nullptr);
//     REQUIRE(msg->getNumMessages() == 3);
//     for(const auto& name : {"rgb", "left", "right"}) {
//         REQUIRE(msg->get(name) != nullptr);
//     }
// }

// // TODO(asahtik): Does not work on windows[issue](https://github.com/luxonis/depthai-core/issues/929) is fixed
// TEST_CASE("Sync - demux") {
//     auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
//     auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

//     dai::Pipeline pipeline;
//     auto demux = pipeline.create<dai::node::MessageDemux>();
//     auto sync = pipeline.create<dai::node::Sync>();
//     sync->setSyncThreshold(std::chrono::milliseconds(100));
//     sync->out.link(demux->input);

//     auto inQ1 = sync->inputs["msg1"].createInputQueue();
//     auto inQ2 = sync->inputs["msg2"].createInputQueue();
//     auto outQ1 = demux->outputs["msg1"].createOutputQueue();
//     auto outQ2 = demux->outputs["msg2"].createOutputQueue();
//     pipeline.start();
//     auto buf1 = std::make_shared<dai::Buffer>();
//     std::vector<uint8_t> buf1Data = {1, 2, 3, 4, 5};
//     std::vector<uint8_t> buf2Data = {6, 7, 8, 9, 10};
//     buf1->setData(buf1Data);
//     buf1->setTimestamp(buf1Ts);

//     auto img1 = std::make_shared<dai::ImgFrame>();
//     img1->setData(buf2Data);
//     img1->setTimestamp(buf2Ts);
//     img1->setSize({5, 6});

//     inQ1->send(buf1);
//     inQ2->send(img1);
//     auto out1 = outQ1->get<dai::Buffer>();
//     auto out2 = outQ2->get<dai::ImgFrame>();

//     REQUIRE(out1->getTimestamp() == buf1Ts);
//     REQUIRE(out2->getTimestamp() == buf2Ts);
//     REQUIRE((out1->getData() == buf1Data));
//     REQUIRE((out2->getData() == buf2Data));
//     REQUIRE(out2->getWidth() == 5);
//     REQUIRE(out2->getHeight() == 6);
// }

// TEST_CASE("MessageGroup ping-pong without XLink") {
//     auto buf1Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
//     auto buf2Ts = std::chrono::steady_clock::now() + std::chrono::milliseconds(150);

//     dai::Pipeline pipeline;
//     // Create nodes that are required
//     auto sync = pipeline.create<dai::node::Sync>();
//     sync->setSyncThreshold(std::chrono::milliseconds(100));  // Should let the messages through
//     auto demux = pipeline.create<dai::node::MessageDemux>();
//     sync->out.link(demux->input);

//     auto inQ1 = sync->inputs["msg1"].createInputQueue();
//     auto inQ2 = sync->inputs["msg2"].createInputQueue();
//     auto outQ1 = demux->outputs["msg1"].createOutputQueue();
//     auto outQ2 = demux->outputs["msg2"].createOutputQueue();
//     pipeline.start();
//     // Buffer 1
//     auto buf1 = std::make_shared<dai::Buffer>();
//     std::vector<uint8_t> buf1Data = {1, 2, 3, 4, 5};
//     buf1->setData(buf1Data);
//     buf1->setTimestamp(buf1Ts);

//     // Image Frame
//     auto img1 = std::make_shared<dai::ImgFrame>();
//     std::vector<uint8_t> buf2Data = {6, 7, 8, 9, 10};
//     img1->setData(buf2Data);
//     img1->setTimestamp(buf2Ts);
//     img1->setSize({5, 6});

//     // Sending the data to queues
//     inQ1->send(buf1);
//     inQ2->send(img1);

//     // Receiving the data back
//     auto out1 = outQ1->get<dai::Buffer>();
//     auto out2 = outQ2->get<dai::ImgFrame>();

//     // Verify buffer data
//     REQUIRE(out1 != nullptr);
//     REQUIRE(out2 != nullptr);

//     // Compare timestamps
//     REQUIRE(out1->getTimestamp() == buf1Ts);
//     REQUIRE(out2->getTimestamp() == buf2Ts);

//     // Compare data using dai::span
//     REQUIRE((out1->getData() == buf1Data));
//     REQUIRE((out2->getData() == buf2Data));

//     // Check image frame size
//     REQUIRE(out2->getWidth() == 5);
//     REQUIRE(out2->getHeight() == 6);
// }
