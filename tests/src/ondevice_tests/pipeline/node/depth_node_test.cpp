/**
 * @file depth_node_test.cpp
 * @brief On-device tests for dai::node::Depth (stereo backend selection, lazy wiring, outputs).
 */
#include <catch2/catch_all.hpp>

#include <cstring>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"

using namespace dai;

namespace {

/// True if @p cam is a direct or nested child of @p depth (Depth's nodeMap subtree).
bool cameraInDepthSubtree(const node::Depth& depth, const std::shared_ptr<node::Camera>& cam) {
    for(const auto& n : depth.getAllNodes()) {
        if(n.get() == cam.get()) {
            return true;
        }
    }
    return false;
}

/// Counts Camera nodes under @p depth that use either socket of the stereo pair.
int countStereoCamerasInDepthSubtree(const node::Depth& depth, const StereoPair& pair) {
    int count = 0;
    for(const auto& n : depth.getAllNodes()) {
        if(std::strcmp(n->getName(), node::Camera::NAME) != 0) {
            continue;
        }
        const auto cam = std::static_pointer_cast<node::Camera>(n);
        if(cam->getBoardSocket() == pair.left || cam->getBoardSocket() == pair.right) {
            ++count;
        }
    }
    return count;
}

}  // namespace

TEST_CASE("Depth: host-only pipeline cannot create node") {
    Pipeline pipeline(false);
    REQUIRE(pipeline.getDefaultDevice() == nullptr);
    REQUIRE_THROWS_AS(pipeline.create<node::Depth>(), std::runtime_error);
}

TEST_CASE("Depth: depth/confidence outputs exist before pipeline.build") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
}

TEST_CASE("Depth: first depth() wires backend before pipeline.build") {
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
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());

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

    int nStereoCams = 0;
    for(const auto& n : pipeline.getAllNodes()) {
        if(std::strcmp(n->getName(), node::Camera::NAME) != 0) {
            continue;
        }
        const auto cam = std::static_pointer_cast<node::Camera>(n);
        if(cam->getBoardSocket() == pairs[0].left || cam->getBoardSocket() == pairs[0].right) {
            ++nStereoCams;
        }
    }
    REQUIRE(nStereoCams == 2);
    REQUIRE(countStereoCamerasInDepthSubtree(*depth, pairs[0]) == 0);
}

TEST_CASE("Depth: build reuses stereo cameras created before Depth node") {
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
    const auto& pair = pairs[0];

    auto leftCam = pipeline.create<node::Camera>()->build(pair.left);
    auto rightCam = pipeline.create<node::Camera>()->build(pair.right);

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());

    REQUIRE_FALSE(cameraInDepthSubtree(*depth, leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*depth, rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*depth, pair) == 0);

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

TEST_CASE("Depth: build reuses stereo cameras created after Depth node") {
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
    const auto& pair = pairs[0];

    auto depth = pipeline.create<node::Depth>();
    auto leftCam = pipeline.create<node::Camera>()->build(pair.left);
    auto rightCam = pipeline.create<node::Camera>()->build(pair.right);

    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());

    REQUIRE_FALSE(cameraInDepthSubtree(*depth, leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*depth, rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*depth, pair) == 0);

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
