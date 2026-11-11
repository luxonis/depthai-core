#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

TEST_CASE("Intermediate ToF depth selection survives device serialization", "[ToFConfig]") {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript("while True:\n    node.io['out'].send(node.io['in'].get())\n");
    auto input = script->inputs["in"].createInputQueue();
    auto output = script->outputs["out"].createOutputQueue();
    pipeline.start();

    for(bool enable : {false, true}) {
        for(bool burst : {false, true}) {
            auto config = std::make_shared<dai::ToFConfig>();
            config->setEnableIntermediateDepth(enable);
            config->enableBurstMode = burst;
            config->enableFPPNCorrection = true;
            config->enablePhaseUnwrapping = true;
            config->phaseUnwrapErrorThreshold = 137;
            config->setMedianFilter(dai::filters::params::MedianFilter::KERNEL_7x7);
            input->send(config);

            bool timedOut = false;
            const auto received = output->get<dai::ToFConfig>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(received != nullptr);
            REQUIRE(dai::utility::serialize(*received) == dai::utility::serialize(*config));
        }
    }
}
