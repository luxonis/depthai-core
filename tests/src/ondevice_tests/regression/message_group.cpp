#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"

TEST_CASE("MessageGroup") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Create Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
        while (True):
            msggrp = node.io['in'].get()
            node.io['out'].send(msggrp)
    )");

    // Create input and output queues
    auto inputQueue = script->inputs["in"].createInputQueue();
    auto outputQueue = script->outputs["out"].createOutputQueue();

    pipeline.start();
    constexpr int WIDTH = 1000;
    constexpr int HEIGHT = 1000;
    constexpr int COUNT = 3;
    for(int i = 0; i < 10; i++) {
        std::vector<std::shared_ptr<dai::ImgFrame>> msgs;
        for(int i = 0; i < COUNT; ++i) {
            auto img = std::make_shared<dai::ImgFrame>();
            img->setData(std::vector<std::uint8_t>(WIDTH * HEIGHT, 128));
            img->setType(dai::ImgFrame::Type::GRAY8);
            msgs.push_back(img);
        }

        auto messageGroup = std::make_shared<dai::MessageGroup>();
        for(int i = 0; i < COUNT; ++i) {
            messageGroup->add("Message - " + std::to_string(i), msgs[i]);
        }

        inputQueue->send(messageGroup);

        // Receive message group
        auto receivedGroup = outputQueue->get<dai::MessageGroup>();
        REQUIRE(receivedGroup != nullptr);
        REQUIRE(receivedGroup->getNumMessages() == COUNT);
    }
}
