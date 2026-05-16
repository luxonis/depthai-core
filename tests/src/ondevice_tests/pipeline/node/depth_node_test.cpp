/**
 * @file depth_node_test.cpp
 * @brief On-device tests for `dai::node::Depth` (AUTO resolution, explicit algorithms, lazy wiring).
 */
#include <catch2/catch_all.hpp>

#include <chrono>
#include <cstring>
#include <optional>

#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"

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

bool deviceReportsTofSensor(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return false;
    }
    try {
        for(const auto& cf : device->getConnectedCameraFeatures()) {
            for(const auto t : cf.supportedTypes) {
                if(t == CameraSensorType::TOF) {
                    return true;
                }
            }
        }
    } catch(...) {
        return false;
    }
    return false;
}

/** A connected camera socket that is not part of the given stereo pair, if any. */
std::optional<CameraBoardSocket> socketOutsideStereoPair(const std::shared_ptr<Device>& device, const StereoPair& pair) {
    if(device == nullptr) {
        return std::nullopt;
    }
    try {
        for(const auto& cf : device->getConnectedCameraFeatures()) {
            if(cf.socket == CameraBoardSocket::AUTO) {
                continue;
            }
            if(cf.socket != pair.left && cf.socket != pair.right) {
                return cf.socket;
            }
        }
    } catch(...) {
        return std::nullopt;
    }
    return std::nullopt;
}

/** Depth wires exactly one direct child for the active algorithm (not nested NAS internals). */
void requireDepthSingleBackendChild(const node::Depth& depth, const char* expectedNodeName) {
    const auto& children = depth.getNodeMap();
    REQUIRE(children.size() == 1);
    REQUIRE(std::strcmp(children[0]->getName(), expectedNodeName) == 0);
}

std::shared_ptr<Device> requireDefaultDevice(Pipeline& pipeline) {
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        SKIP("Skipping Depth test: no device connected.");
    }
    return device;
}

StereoPair requireFirstStereoPairForTest(const std::shared_ptr<Device>& device) {
    const auto pairs = device->getStereoPairs();
    if(pairs.empty()) {
        SKIP("Skipping Depth test: device has no stereo pair.");
    }
    return pairs[0];
}

void startPipelineAndRequireFirstFrames(Pipeline& pipeline, const std::shared_ptr<node::Depth>& depth) {
    auto depthQueue = depth->depth().createOutputQueue();
    auto confidenceQueue = depth->confidence().createOutputQueue();

    pipeline.start();

    bool timedOut = false;
    auto depthFrame = depthQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depthFrame != nullptr);

    timedOut = false;
    auto confidenceFrame = confidenceQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(confidenceFrame != nullptr);
}

}  // namespace

TEST_CASE("Depth: host-only pipeline cannot create node") {
    Pipeline pipeline(false);
    REQUIRE(pipeline.getDefaultDevice() == nullptr);
    REQUIRE_THROWS_AS(pipeline.create<node::Depth>(), std::runtime_error);
}

TEST_CASE("Depth: depth/confidence outputs exist before pipeline.build") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
}

TEST_CASE("Depth: create(device, algorithm) exposes algorithm via getAlgorithm") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
    REQUIRE(depth->getAlgorithm() == node::Depth::Algorithm::TOF);
}

TEST_CASE("Depth: AUTO selects backend by platform and sensors") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    const auto platform = device->getPlatform();
    const auto autoUsesTof = platform == Platform::RVC2 && deviceReportsTofSensor(device);
    if(!autoUsesTof) {
        (void)requireFirstStereoPairForTest(device);
    }

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));

    if(platform == Platform::RVC4) {
        requireDepthSingleBackendChild(*depth, "NeuralDepth");
    } else if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        requireDepthSingleBackendChild(*depth, "ToF");
    } else {
        requireDepthSingleBackendChild(*depth, "StereoDepth");
    }
}

TEST_CASE("Depth: explicit STEREO on RVC4 uses StereoDepth") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping Depth test: not RVC4.");
    }
    (void)requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::STEREO);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "StereoDepth");
}

TEST_CASE("Depth: GPU_STEREO requires RVC4") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() == Platform::RVC4) {
        SKIP("Skipping negative GPU_STEREO test on RVC4.");
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::GPU_STEREO);
    REQUIRE_THROWS((void)&depth->depth());
}

// Mirrors Depth::validateAlgorithm(TOF): requires CameraSensorType::TOF in getConnectedCameraFeatures().
TEST_CASE("Depth: TOF requires connected ToF camera") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(deviceReportsTofSensor(device)) {
        SKIP("Skipping negative TOF test: device reports a ToF sensor.");
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
    REQUIRE_THROWS((void)&depth->depth());
}

TEST_CASE("Depth: build reuses stereo cameras created before Depth node") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    const auto pair = requireFirstStereoPairForTest(device);

    auto leftCam = pipeline.create<node::Camera>()->build(pair.left);
    auto rightCam = pipeline.create<node::Camera>()->build(pair.right);

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));

    REQUIRE_FALSE(cameraInDepthSubtree(*depth, leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*depth, rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*depth, pair) == 0);

    const auto platform = device->getPlatform();
    if(platform == Platform::RVC4) {
        requireDepthSingleBackendChild(*depth, "NeuralDepth");
    } else if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        requireDepthSingleBackendChild(*depth, "ToF");
    } else {
        requireDepthSingleBackendChild(*depth, "StereoDepth");
    }
}

TEST_CASE("Depth: explicit NEURAL wires NeuralDepth when device supports it") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(!device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth test: device does not support NeuralDepth.");
    }
    (void)requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "NeuralDepth");
}

TEST_CASE("Depth: NEURAL_ASSISTED_STEREO rejected off RVC4") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() == Platform::RVC4) {
        SKIP("Skipping Depth test: RVC4 device (negative NAS test not applicable).");
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_THROWS((void)&depth->depth());
}

TEST_CASE("Depth: NEURAL_ASSISTED_STEREO wires NeuralAssistedStereo on RVC4") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping Depth test: not RVC4.");
    }
    if(!device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth test: device does not support NeuralDepth.");
    }
    (void)requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "NeuralAssistedStereo");
}

TEST_CASE("Depth: TOF algorithm wires ToF backend when ToF sensor present") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(!deviceReportsTofSensor(device)) {
        SKIP("Skipping Depth test: no ToF sensor reported.");
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "ToF");
}

TEST_CASE("Depth: GPU_STEREO wires GPUStereo on RVC4 when build and device allow") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping Depth test: not RVC4.");
    }
    (void)requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::GPU_STEREO);
    try {
        (void)&depth->depth();
    } catch(const std::exception& ex) {
        SKIP(std::string("Skipping GPU_STEREO positive test: ") + ex.what());
    }
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "GPUStereo");
}

TEST_CASE("Depth: stereo cameras created after Depth still reuse pipeline cameras") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    const auto pair = requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>();
    auto leftCam = pipeline.create<node::Camera>()->build(pair.left);
    auto rightCam = pipeline.create<node::Camera>()->build(pair.right);

    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));

    REQUIRE_FALSE(cameraInDepthSubtree(*depth, leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*depth, rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*depth, pair) == 0);

    const auto platform = device->getPlatform();
    if(platform == Platform::RVC4) {
        requireDepthSingleBackendChild(*depth, "NeuralDepth");
    } else if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        requireDepthSingleBackendChild(*depth, "ToF");
    } else {
        requireDepthSingleBackendChild(*depth, "StereoDepth");
    }
}

TEST_CASE("Depth: pipeline with SystemLogger and optional third camera still builds") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    const auto pair = requireFirstStereoPairForTest(device);

    pipeline.create<node::SystemLogger>();
    if(const auto extra = socketOutsideStereoPair(device, pair)) {
        REQUIRE_NOTHROW(pipeline.create<node::Camera>()->build(*extra));
    }

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
}
