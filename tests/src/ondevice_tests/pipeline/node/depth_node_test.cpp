/**
 * @file depth_node_test.cpp
 * @brief On-device tests for dai::node::Depth (algorithm selection, lazy wiring).
 */
#include <catch2/catch_all.hpp>

#include <cstring>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"

using namespace dai;

namespace {

bool cameraInDepthSubtree(const node::Depth& depth, const std::shared_ptr<node::Camera>& cam) {
    for(const auto& n : depth.getAllNodes()) {
        if(n.get() == cam.get()) {
            return true;
        }
    }
    return false;
}

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

TEST_CASE("Depth: create(device, algorithm) exposes algorithm via getAlgorithm") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
    REQUIRE(depth->getAlgorithm() == node::Depth::Algorithm::TOF);
}

TEST_CASE("Depth: AUTO wires NeuralDepth on RVC4 and StereoDepth otherwise") {
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
}

TEST_CASE("Depth: explicit STEREO on RVC4 uses StereoDepth") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    if(device->getPlatform() != Platform::RVC4) {
        WARN("Skipping Depth test: not RVC4.");
        return;
    }
    const auto pairs = device->getStereoPairs();
    if(pairs.empty()) {
        WARN("Skipping Depth test: device has no stereo pair.");
        return;
    }

    auto depth = pipeline.create<node::Depth>()->setAlgorithm(node::Depth::Algorithm::STEREO);
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());
    REQUIRE(depth->getStereoDepth() != nullptr);
    REQUIRE(depth->getNeuralDepth() == nullptr);
}

TEST_CASE("Depth: GPU_STEREO requires RVC4") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    if(device->getPlatform() == Platform::RVC4) {
        WARN("Skipping negative GPU_STEREO test on RVC4.");
        return;
    }
    auto depth = pipeline.create<node::Depth>()->setAlgorithm(node::Depth::Algorithm::GPU_STEREO);
    REQUIRE_THROWS((void)&depth->depth());
}

// Mirrors Depth::validateAlgorithm(TOF): requires CameraSensorType::TOF in getConnectedCameraFeatures().
TEST_CASE("Depth: TOF requires connected ToF camera") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    bool hasTof = false;
    for(const auto& cf : device->getConnectedCameraFeatures()) {
        for(const auto t : cf.supportedTypes) {
            if(t == CameraSensorType::TOF) {
                hasTof = true;
                break;
            }
        }
        if(hasTof) {
            break;
        }
    }
    if(hasTof) {
        WARN("Skipping negative TOF test: device reports a ToF sensor.");
        return;
    }
    auto depth = pipeline.create<node::Depth>()->setAlgorithm(node::Depth::Algorithm::TOF);
    REQUIRE_THROWS((void)&depth->depth());
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
    } else {
        REQUIRE(depth->getStereoDepth() != nullptr);
    }
}
