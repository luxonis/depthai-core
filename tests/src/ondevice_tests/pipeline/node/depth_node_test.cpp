/**
 * @file depth_node_test.cpp
 * @brief On-device tests for `dai::node::Depth` (algorithms, lazy wiring, user camera coexistence).
 */
#include <catch2/catch_all.hpp>

#include <cmath>
#include <cctype>
#include <chrono>
#include <cstring>
#include <optional>
#include <utility>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/ToF.hpp"

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

/** Depth wires exactly one direct child for the active algorithm (not nested internals). */
void requireDepthSingleBackendChild(const node::Depth& depth, const char* expectedNodeName) {
    const auto& children = depth.getNodeMap();
    REQUIRE(children.size() == 1);
    const auto& child = children[0];
    // DeviceNodeGroup composites (NAS, ToF) report getName() as "DeviceNodeGroup".
    if(std::strcmp(expectedNodeName, "NeuralAssistedStereo") == 0) {
        REQUIRE(std::dynamic_pointer_cast<node::NeuralAssistedStereo>(child) != nullptr);
        return;
    }
    if(std::strcmp(expectedNodeName, "ToF") == 0) {
        REQUIRE(std::dynamic_pointer_cast<node::ToF>(child) != nullptr);
        return;
    }
    REQUIRE(std::strcmp(child->getName(), expectedNodeName) == 0);
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

void requireDepthAutoBackend(const node::Depth& depth, const std::shared_ptr<Device>& device) {
    const auto platform = device->getPlatform();
    if(platform == Platform::RVC4) {
        requireDepthSingleBackendChild(depth, "NeuralDepth");
    } else if(platform == Platform::RVC2 && deviceReportsTofSensor(device)) {
        requireDepthSingleBackendChild(depth, "ToF");
    } else {
        requireDepthSingleBackendChild(depth, "StereoDepth");
    }
}

/** Stops the pipeline on scope exit if a prior start() left it running. */
struct PipelineStopGuard {
    Pipeline& pipeline;
    explicit PipelineStopGuard(Pipeline& p) : pipeline(p) {}
    ~PipelineStopGuard() {
        if(pipeline.isRunning()) {
            pipeline.stop();
        }
    }
    PipelineStopGuard(const PipelineStopGuard&) = delete;
    PipelineStopGuard& operator=(const PipelineStopGuard&) = delete;
};

/** Host queues must exist before pipeline.start() (start() calls build()). */
void requireDepthPipelineBuilds(Pipeline& pipeline, const std::shared_ptr<node::Depth>& depth) {
    REQUIRE_NOTHROW((void)&depth->depth());
    if(depth->hasConfidence()) {
        REQUIRE_NOTHROW((void)&depth->confidence());
    }
    REQUIRE_NOTHROW(pipeline.build());
}

void startPipelineAndRequireFirstFrames(Pipeline& pipeline, const std::shared_ptr<node::Depth>& depth) {
    PipelineStopGuard guard(pipeline);

    auto depthQueue = depth->depth().createOutputQueue();
    std::shared_ptr<MessageQueue> confidenceQueue;
    if(depth->hasConfidence()) {
        confidenceQueue = depth->confidence().createOutputQueue();
    }

    pipeline.start();

    bool timedOut = false;
    auto depthFrame = depthQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depthFrame != nullptr);

    if(confidenceQueue == nullptr) {
        return;
    }

    timedOut = false;
    auto confidenceFrame = confidenceQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(confidenceFrame != nullptr);
}

constexpr std::pair<uint32_t, uint32_t> kUserStereoSensorResolution{1280, 800};
constexpr float kUserStereoFps = 30.0f;
constexpr float kDepthStereoFps = 15.0f;
constexpr float kBenchmarkFpsMarginRatio = 0.15f;
constexpr std::pair<uint32_t, uint32_t> kStereoDepthMonoSize{640, 400};

constexpr DeviceModelZoo kDepthDefaultNeuralModel = DeviceModelZoo::NEURAL_DEPTH_SMALL;

void skipUnlessUserStereoDepthScenario(const std::shared_ptr<Device>& device) {
    if(device->getPlatform() == Platform::RVC2 && deviceReportsTofSensor(device)) {
        SKIP("Skipping Depth user-camera test: RVC2 AUTO selects ToF, not user stereo cameras.");
    }
    (void)requireFirstStereoPairForTest(device);
    if(device->getPlatform() == Platform::RVC4 && !device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth user-camera test: device does not support NeuralDepth.");
    }
}

std::pair<std::shared_ptr<node::Camera>, std::shared_ptr<node::Camera>> buildUserStereoCamerasOrSkip(Pipeline& pipeline,
                                                                                                    const StereoPair& pair) {
    try {
        auto leftCam = pipeline.create<node::Camera>()->build(pair.left, kUserStereoSensorResolution, kUserStereoFps);
        auto rightCam = pipeline.create<node::Camera>()->build(pair.right, kUserStereoSensorResolution, kUserStereoFps);
        return {leftCam, rightCam};
    } catch(const std::exception& ex) {
        SKIP(std::string("Skipping Depth user-camera test: cannot build stereo cameras at 1280x800@30: ") + ex.what());
    }
    return {{}, {}};  // unreachable (SKIP throws)
}

void requireBenchmarkFpsNear(const std::shared_ptr<MessageQueue>& reportQueue, float expectedFps, int samples = 3) {
    REQUIRE(reportQueue != nullptr);
    (void)reportQueue->get<BenchmarkReport>();  // warmup
    const auto margin = expectedFps * kBenchmarkFpsMarginRatio;
    for(int i = 0; i < samples; ++i) {
        bool timedOut = false;
        auto report = reportQueue->get<BenchmarkReport>(std::chrono::seconds(30), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(report != nullptr);
        REQUIRE(report->fps == Catch::Approx(expectedFps).margin(margin));
    }
}

struct UserDepthCameraBenchSetup {
    std::shared_ptr<node::Camera> leftCam;
    std::shared_ptr<node::Camera> rightCam;
    std::shared_ptr<MessageQueue> userBenchQueue;
    std::shared_ptr<MessageQueue> depthBenchQueue;
    std::shared_ptr<MessageQueue> userFrameQueue;
    std::shared_ptr<MessageQueue> depthFrameQueue;
    std::shared_ptr<node::Depth> depth;
};

UserDepthCameraBenchSetup wireUserStereoCamerasAndDepth(Pipeline& pipeline,
                                                        const StereoPair& pair,
                                                        std::optional<float> depthRequestedFps) {
    UserDepthCameraBenchSetup setup;
    std::tie(setup.leftCam, setup.rightCam) = buildUserStereoCamerasOrSkip(pipeline, pair);

    auto* userLeftOut =
        setup.leftCam->requestOutput(kUserStereoSensorResolution, std::nullopt, ImgResizeMode::CROP, kUserStereoFps);
    REQUIRE(userLeftOut != nullptr);
    setup.userFrameQueue = userLeftOut->createOutputQueue();

    auto userBench = pipeline.create<node::BenchmarkIn>();
    userBench->sendReportEveryNMessages(static_cast<uint32_t>(std::lround(kUserStereoFps * 2)));
    userLeftOut->link(userBench->input);
    setup.userBenchQueue = userBench->report.createOutputQueue();

    setup.depth = pipeline.create<node::Depth>();
    if(depthRequestedFps.has_value()) {
        setup.depth->build(*depthRequestedFps);
    }

    const float depthBenchTargetFps = depthRequestedFps.value_or(kUserStereoFps);
    auto depthBench = pipeline.create<node::BenchmarkIn>();
    depthBench->sendReportEveryNMessages(static_cast<uint32_t>(std::lround(depthBenchTargetFps * 2)));
    setup.depth->depth().link(depthBench->input);
    setup.depthBenchQueue = depthBench->report.createOutputQueue();
    setup.depthFrameQueue = setup.depth->depth().createOutputQueue();

    return setup;
}

void requireUserAndDepthFrameSizes(const std::shared_ptr<Device>& device,
                                   const std::shared_ptr<node::Depth>& depth,
                                   const std::shared_ptr<MessageQueue>& userFrameQueue,
                                   const std::shared_ptr<MessageQueue>& depthFrameQueue) {
    bool timedOut = false;
    auto userFrame = userFrameQueue->get<ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(userFrame != nullptr);
    REQUIRE(userFrame->getWidth() == static_cast<int>(kUserStereoSensorResolution.first));
    REQUIRE(userFrame->getHeight() == static_cast<int>(kUserStereoSensorResolution.second));

    timedOut = false;
    auto depthFrame = depthFrameQueue->get<ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depthFrame != nullptr);
    REQUIRE((depthFrame->getWidth() != static_cast<int>(kUserStereoSensorResolution.first)
             || depthFrame->getHeight() != static_cast<int>(kUserStereoSensorResolution.second)));

    if(device->getPlatform() == Platform::RVC4) {
        const auto [expectedW, expectedH] = node::NeuralDepth::getInputSize(kDepthDefaultNeuralModel);
        requireDepthSingleBackendChild(*depth, "NeuralDepth");
        REQUIRE(depthFrame->getWidth() == expectedW);
        REQUIRE(depthFrame->getHeight() == expectedH);
    } else {
        requireDepthSingleBackendChild(*depth, "StereoDepth");
        REQUIRE(depthFrame->getWidth() == static_cast<int>(kStereoDepthMonoSize.first));
        REQUIRE(depthFrame->getHeight() == static_cast<int>(kStereoDepthMonoSize.second));
    }
}

void runUserCameraDepthBenchmarkTest(Pipeline& pipeline,
                                     const std::shared_ptr<Device>& device,
                                     const StereoPair& pair,
                                     std::optional<float> depthRequestedFps,
                                     bool checkFrameSizes) {
    PipelineStopGuard guard(pipeline);
    auto setup = wireUserStereoCamerasAndDepth(pipeline, pair, depthRequestedFps);

    REQUIRE_FALSE(cameraInDepthSubtree(*setup.depth, setup.leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*setup.depth, setup.rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*setup.depth, pair) == 0);

    pipeline.build();
    pipeline.start();

    requireBenchmarkFpsNear(setup.userBenchQueue, kUserStereoFps);
    const float expectedDepthFps = depthRequestedFps.value_or(kUserStereoFps);
    requireBenchmarkFpsNear(setup.depthBenchQueue, expectedDepthFps);

    if(checkFrameSizes) {
        requireUserAndDepthFrameSizes(device, setup.depth, setup.userFrameQueue, setup.depthFrameQueue);
    }

    requireDepthAutoBackend(*setup.depth, device);
}

}  // namespace

TEST_CASE("Depth: host-only pipeline cannot create node") {
    Pipeline pipeline(false);
    REQUIRE(pipeline.getDefaultDevice() == nullptr);
    REQUIRE_THROWS_AS(pipeline.create<node::Depth>(), std::runtime_error);
}

TEST_CASE("Depth: depth/confidence outputs exist before pipeline.build") {
    Pipeline pipeline;
    (void)requireDefaultDevice(pipeline);
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
    (void)requireDefaultDevice(pipeline);
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

    requireDepthAutoBackend(*depth, device);
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

    requireDepthAutoBackend(*depth, device);
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
    if(!deviceReportsGpuStereoBoardRevision(device)) {
        SKIP("Skipping GPU_STEREO positive test: board revision below R9 (or boardRev unavailable).");
    }

    auto depth = pipeline.create<node::Depth>(node::Depth::Algorithm::GPU_STEREO);
    try {
        (void)&depth->depth();
    } catch(const std::exception& ex) {
        SKIP(std::string("Skipping GPU_STEREO positive test: ") + ex.what());
    }
    REQUIRE_FALSE(depth->hasConfidence());
    REQUIRE_THROWS_WITH((void)&depth->confidence(), Catch::Matchers::ContainsSubstring("GPUStereo"));
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

    requireDepthAutoBackend(*depth, device);
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
    REQUIRE_NOTHROW(requireDepthPipelineBuilds(pipeline, depth));
}

TEST_CASE("Depth: user stereo cameras keep 30 FPS with AUTO and no build(fps)") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthBenchmarkTest(pipeline, device, pair, std::nullopt, false));
}

TEST_CASE("Depth: user cameras stay 30 FPS while depth build(fps) runs at 15 FPS") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthBenchmarkTest(pipeline, device, pair, kDepthStereoFps, false));
}

TEST_CASE("Depth: user camera resolution unchanged; depth uses backend size") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthBenchmarkTest(pipeline, device, pair, std::nullopt, true));
}
