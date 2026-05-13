/**
 * @file focused_depth_node_test.cpp
 * @brief On-device smoke tests for dai::node::FocusedDepth (RVC4).
 */
#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/FocusedDepth.hpp"

using namespace dai;

TEST_CASE("FocusedDepth: host-only pipeline cannot create node") {
    Pipeline pipeline(false);
    REQUIRE(pipeline.getDefaultDevice() == nullptr);
    REQUIRE_THROWS_AS(pipeline.create<node::FocusedDepth>(), std::runtime_error);
}

TEST_CASE("FocusedDepth: non-RVC4 rejects build") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping FocusedDepth test: no device connected.");
        return;
    }
    if(device->getPlatform() == Platform::RVC4) {
        WARN("Skipping negative FocusedDepth test on RVC4.");
        return;
    }
    auto fd = pipeline.create<node::FocusedDepth>();
    REQUIRE_THROWS((void)&fd->depth());
}

TEST_CASE("FocusedDepth: RVC4 builds depth and confidence outputs") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping FocusedDepth test: no device connected.");
        return;
    }
    if(device->getPlatform() != Platform::RVC4) {
        WARN("Skipping FocusedDepth test: not RVC4.");
        return;
    }
    const auto pairs = device->getStereoPairs();
    if(pairs.empty()) {
        WARN("Skipping FocusedDepth test: no stereo pair.");
        return;
    }

    auto fd = pipeline.create<node::FocusedDepth>();
    REQUIRE_NOTHROW((void)&fd->depth());
    REQUIRE_NOTHROW((void)&fd->confidence());
    REQUIRE_NOTHROW(pipeline.build());

    const bool stereo = fd->getStereoDepth() != nullptr;
    const bool neural = fd->getNeuralDepth() != nullptr;
    REQUIRE((stereo || neural));
    REQUIRE(!(stereo && neural));
}
