/**
 * @file depth_node_test.cpp
 * @brief On-device tests for `dai::node::Depth` (AUTO resolution, explicit algorithms, lazy wiring).
 */
#include <catch2/catch_all.hpp>

#include <cctype>
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

std::optional<int> readDecimalMajorAt(const std::string& s, std::size_t& i) {
    if(i >= s.size() || std::isdigit(static_cast<unsigned char>(s[i])) == 0) {
        return std::nullopt;
    }
    int major = 0;
    while(i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) {
        major = major * 10 + (s[i] - '0');
        ++i;
    }
    return major;
}

std::optional<int> parseBoardRevisionMajor(const std::string& boardRev) {
    if(boardRev.empty()) {
        return std::nullopt;
    }
    std::size_t i = 0;
    while(i < boardRev.size() && std::isspace(static_cast<unsigned char>(boardRev[i])) != 0) {
        ++i;
    }
    if(i >= boardRev.size()) {
        return std::nullopt;
    }
    const char lead = static_cast<char>(std::tolower(static_cast<unsigned char>(boardRev[i])));
    ++i;
    if(lead == 'p') {
        auto major = readDecimalMajorAt(boardRev, i);
        if(major && i < boardRev.size() && std::tolower(static_cast<unsigned char>(boardRev[i])) == 'd') {
            return major;
        }
        return std::nullopt;
    }
    if(lead == 'r') {
        while(i < boardRev.size() && std::isspace(static_cast<unsigned char>(boardRev[i])) != 0) {
            ++i;
        }
        return readDecimalMajorAt(boardRev, i);
    }
    return std::nullopt;
}

bool deviceReportsGpuStereoBoardRevision(const std::shared_ptr<Device>& device) {
    if(device == nullptr || device->getPlatform() != Platform::RVC4) {
        return false;
    }
    try {
        const auto major = parseBoardRevisionMajor(device->readCalibrationOrDefault().getEepromData().boardRev);
        return major && *major >= 9;
    } catch(...) {
        return false;
    }
}

bool deviceSupportsGpuStereoForAuto(const std::shared_ptr<Device>& device) {
    if(device == nullptr || device->getPlatform() != Platform::RVC4) {
        return false;
    }
    if(device->getStereoPairs().empty()) {
        return false;
    }
    return deviceReportsGpuStereoBoardRevision(device);
}

const char* expectedAutoBackendName(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return "StereoDepth";
    }
    const auto platform = device->getPlatform();
    if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        return "ToF";
    }
    if(platform == Platform::RVC4) {
#if defined(DEPTHAI_ENABLE_KOMPUTE)
        if(deviceSupportsGpuStereoForAuto(device)) {
            return "GPUStereo";
        }
#endif
        return "NeuralDepth";
    }
    return "StereoDepth";
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

/** Depth wires exactly one backend device node. */
void requireDepthSingleBackendChild(const node::Depth& depth, const char* expectedNodeName) {
    const auto& children = depth.getNodeMap();
    REQUIRE(children.size() == 1);
    REQUIRE(std::strcmp(children[0]->getName(), expectedNodeName) == 0);
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
    if(depth->hasConfidence()) {
        REQUIRE_NOTHROW((void)&depth->confidence());
    } else {
        REQUIRE_THROWS_WITH((void)&depth->confidence(), Catch::Matchers::ContainsSubstring("GPUStereo"));
    }
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

TEST_CASE("Depth: AUTO selects backend by platform and sensors") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    const auto platform = device->getPlatform();
    const auto pairs = device->getStereoPairs();

    if(platform == Platform::RVC4) {
        if(pairs.empty()) {
            WARN("Skipping Depth test: device has no stereo pair.");
            return;
        }
    } else if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        // AUTO -> ToF: no stereo pair required.
    } else {
        if(pairs.empty()) {
            WARN("Skipping Depth test: device has no stereo pair.");
            return;
        }
    }

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());

    requireDepthSingleBackendChild(*depth, expectedAutoBackendName(device));
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

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::STEREO);
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW(pipeline.build());
    requireDepthSingleBackendChild(*depth, "StereoDepth");
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
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::GPU_STEREO);
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
    if(deviceReportsTofSensor(device)) {
        WARN("Skipping negative TOF test: device reports a ToF sensor.");
        return;
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
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

    requireDepthSingleBackendChild(*depth, expectedAutoBackendName(device));
}

TEST_CASE("Depth: explicit NEURAL wires NeuralDepth when device supports it") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping Depth test: device does not support NeuralDepth.");
        return;
    }
    const auto pairs = device->getStereoPairs();
    if(pairs.empty()) {
        WARN("Skipping Depth test: device has no stereo pair.");
        return;
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL);
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
    REQUIRE_NOTHROW(pipeline.build());
    requireDepthSingleBackendChild(*depth, "NeuralDepth");
}

TEST_CASE("Depth: NEURAL_ASSISTED_STEREO rejected off RVC4") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    if(device->getPlatform() == Platform::RVC4) {
        WARN("Skipping Depth test: RVC4 device (negative NAS test not applicable).");
        return;
    }
    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_THROWS((void)&depth->depth());
}

TEST_CASE("Depth: NEURAL_ASSISTED_STEREO wires NeuralAssistedStereo on RVC4") {
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
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping Depth test: device does not support NeuralDepth.");
        return;
    }
    if(device->getStereoPairs().empty()) {
        WARN("Skipping Depth test: device has no stereo pair.");
        return;
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
    REQUIRE_NOTHROW(pipeline.build());
    requireDepthSingleBackendChild(*depth, "NeuralAssistedStereo");
}

TEST_CASE("Depth: TOF algorithm wires ToF backend when ToF sensor present") {
    Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        WARN("Skipping Depth test: no device connected.");
        return;
    }
    if(!deviceReportsTofSensor(device)) {
        WARN("Skipping Depth test: no ToF sensor reported.");
        return;
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::TOF);
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
    REQUIRE_NOTHROW(pipeline.build());
    requireDepthSingleBackendChild(*depth, "ToF");
}

TEST_CASE("Depth: GPU_STEREO wires GPUStereo on RVC4 when build and device allow") {
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
    if(device->getStereoPairs().empty()) {
        WARN("Skipping Depth test: device has no stereo pair.");
        return;
    }
    if(!deviceReportsGpuStereoBoardRevision(device)) {
        WARN("Skipping GPU_STEREO positive test: board revision below R9 (or boardRev unavailable).");
        return;
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::GPU_STEREO);
    try {
        (void)&depth->depth();
    } catch(const std::exception& ex) {
        WARN("Skipping GPU_STEREO positive test: " << ex.what());
        return;
    }
    REQUIRE_FALSE(depth->hasConfidence());
    REQUIRE_THROWS_WITH((void)&depth->confidence(), Catch::Matchers::ContainsSubstring("GPUStereo"));
    REQUIRE_NOTHROW(pipeline.build());
    requireDepthSingleBackendChild(*depth, "GPUStereo");
}

TEST_CASE("Depth: stereo cameras created after Depth still reuse pipeline cameras") {
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

    requireDepthSingleBackendChild(*depth, expectedAutoBackendName(device));
}

TEST_CASE("Depth: pipeline with SystemLogger and optional third camera still builds") {
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

    pipeline.create<node::SystemLogger>();
    if(const auto extra = socketOutsideStereoPair(device, pair)) {
        REQUIRE_NOTHROW(pipeline.create<node::Camera>()->build(*extra));
    }

    auto depth = pipeline.create<node::Depth>();
    REQUIRE_NOTHROW((void)&depth->depth());
    REQUIRE_NOTHROW((void)&depth->confidence());
    REQUIRE_NOTHROW(pipeline.build());
}
