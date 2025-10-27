#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

TEST_CASE("Old API Test") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    while True:
        inMessage = node.io["log"].get()
        if inMessage is None:
            break
        message = Buffer(10)
        node.io["out"].send(message)
    )");

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    // Send message and wait for response
    in->send(std::make_shared<dai::Buffer>());
    auto output = out->get<dai::Buffer>();

    REQUIRE(output != nullptr);
}

TEST_CASE("New API Test") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    while True:
        inMessage = node.inputs["log"].get()
        if inMessage is None:
            break
        message = Buffer(10)
        node.outputs["out"].send(message)
    )");

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    // Send message and wait for response
    in->send(std::make_shared<dai::Buffer>());
    auto output = out->get<dai::Buffer>();

    REQUIRE(output != nullptr);
}
