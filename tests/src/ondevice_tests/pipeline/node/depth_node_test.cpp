/**
 * @file depth_node_test.cpp
 * @brief On-device tests for dai::node::Depth (stereo backend selection, build lifecycle, outputs).
 */
#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Depth.hpp"

using namespace dai;

TEST_CASE("Depth: host-only pipeline cannot create node") {
    Pipeline pipeline(false);
    REQUIRE(pipeline.getDefaultDevice() == nullptr);
    REQUIRE_THROWS_AS(pipeline.create<node::Depth>(), std::runtime_error);
}

TEST_CASE("Depth: outputs throw before build") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    auto depth = pipeline.create<node::Depth>();
    REQUIRE_THROWS_AS(depth->depth(), std::runtime_error);
    REQUIRE_THROWS_AS(depth->confidence(), std::runtime_error);
}

TEST_CASE("Depth: build wires backend by platform") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    const auto pairs = device->getStereoPairs();
    if(pairs.empty()) {
        WARN("Skipping Depth test: device has no stereo pair.");
        return;
    }

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW(depth->build());

    const auto platform = device->getPlatform();
    if(platform == Platform::RVC4) {
        REQUIRE(depth->getNeuralDepth() != nullptr);
        REQUIRE(depth->getStereoDepth() == nullptr);
    } else {
        REQUIRE(depth->getStereoDepth() != nullptr);
        REQUIRE(depth->getNeuralDepth() == nullptr);
    }

    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
}

TEST_CASE("Depth: double build throws") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr || device->getStereoPairs().empty()) {
        WARN("Skipping Depth double-build test.");
        return;
    }
    auto depth = pipeline.create<node::Depth>();
    depth->build();
    REQUIRE_THROWS_AS(depth->build(), std::runtime_error);
}
