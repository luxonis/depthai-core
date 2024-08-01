#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

TEST_CASE("SpatialDetectionNetwork sets device for all subnodes") {
    // Create pipeline
    dai::Pipeline p;
    auto nn = p.create<dai::node::SpatialDetectionNetwork>();
    auto pipelineDevice = p.getDefaultDevice();

    REQUIRE(nn->getDevice() == pipelineDevice);
    REQUIRE(nn->neuralNetwork->getDevice() == pipelineDevice);
    REQUIRE(nn->detectionParser->getDevice() == pipelineDevice);
}
