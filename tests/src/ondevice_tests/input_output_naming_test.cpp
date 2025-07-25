
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "depthai/depthai.hpp"

class TestNode : public dai::node::CustomThreadedNode<TestNode> {
   public:
    void run() override {
        while(isRunning()) {
            // do nothing
        }
    }
};

TEST_CASE("input output auto naming test") {
    // Create pipeline
    dai::Pipeline pipeline;

    auto node = pipeline.create<TestNode>();
    dai::DeviceNode::InputDescription desc;
    auto input = std::make_shared<dai::Node::Input>(*node, desc);

    REQUIRE(input->getName() == "_HostNode_input_0");

    auto input2 = std::make_shared<dai::Node::Input>(*node, desc);

    REQUIRE(input2->getName() == "_HostNode_input_1");

    dai::Node::OutputDescription outputDesc;
    auto output = std::make_shared<dai::Node::Output>(*node, outputDesc);
    REQUIRE(output->getName() == "_HostNode_output_0");

    auto output2 = std::make_shared<dai::Node::Output>(*node, outputDesc);
    REQUIRE(output2->getName() == "_HostNode_output_1");
}
