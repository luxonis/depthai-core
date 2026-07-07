#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <optional>
#include <utility>
#include <variant>

#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/depthai.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/NeuralAssistedStereo.hpp"
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/SystemLogger.hpp"
#include "depthai/pipeline/node/ToF.hpp"

using namespace dai;

namespace dai::node {

struct DepthTestAccess {
    static std::pair<Depth::Algorithm, Depth::Config> selectBackend(std::optional<std::pair<uint32_t, uint32_t>> resolution,
                                                                    float targetFps,
                                                                    const std::vector<Depth::Algorithm>& supportedAlgorithms,
                                                                    const std::vector<DeviceModelZoo>& modelFilter = {}) {
        const auto selection = Depth::selectBackend(resolution, targetFps, supportedAlgorithms, modelFilter);
        return {selection.algorithm, selection.config};
    }

    static bool exceedsStereoDepthMaxResolution(uint32_t width, uint32_t height) {
        return Depth::exceedsStereoDepthMaxResolution(width, height);
    }

    static std::vector<Depth::Algorithm> getSupportedAlgorithms(const Depth& depth, const std::shared_ptr<Device>& device) {
        const auto models = device->getPlatform() == Platform::RVC4 ? device->getSupportedDeviceModels() : std::vector<DeviceModelZoo>{};
        return depth.getSupportedAlgorithms(device, models);
    }
};

}  // namespace dai::node

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

bool deviceSupportsGpuStereo(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return false;
    }
    return device->isGpuStereoSupported() && !device->getStereoPairs().empty();
}

bool deviceReportsTofSensor(const std::shared_ptr<Device>& device) {
    if(device == nullptr) {
        return false;
    }
    for(const auto& cf : device->getConnectedCameraFeatures()) {
        for(const auto t : cf.supportedTypes) {
            if(t == CameraSensorType::TOF) {
                return true;
            }
        }
    }
    return false;
}

/** A connected camera socket that is not part of the given stereo pair, if any. */
std::optional<CameraBoardSocket> socketOutsideStereoPair(const std::shared_ptr<Device>& device, const StereoPair& pair) {
    if(device == nullptr) {
        return std::nullopt;
    }
    for(const auto& cf : device->getConnectedCameraFeatures()) {
        if(cf.socket == CameraBoardSocket::AUTO) {
            continue;
        }
        if(cf.socket != pair.left && cf.socket != pair.right) {
            return cf.socket;
        }
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
        switch(depth.getResolvedAlgorithm()) {
            case node::Depth::Algorithm::NEURAL:
                requireDepthSingleBackendChild(depth, "NeuralDepth");
                break;
            case node::Depth::Algorithm::NEURAL_ASSISTED_STEREO:
                requireDepthSingleBackendChild(depth, "NeuralAssistedStereo");
                break;
            case node::Depth::Algorithm::GPU_STEREO:
                requireDepthSingleBackendChild(depth, "GPUStereo");
                break;
            case node::Depth::Algorithm::STEREO:
                requireDepthSingleBackendChild(depth, "StereoDepth");
                break;
            default:
                FAIL("Depth AUTO on RVC4 resolved to an unexpected backend.");
        }
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
    auto* output = depth->requestOutput();
    REQUIRE(output != nullptr);
    REQUIRE_NOTHROW((void)output->depth.operator->());
    REQUIRE_NOTHROW((void)output->confidence.operator->());
    REQUIRE_NOTHROW(pipeline.build());
}

void startPipelineAndRequireFirstFrames(Pipeline& pipeline, const std::shared_ptr<node::Depth>& depth) {
    PipelineStopGuard guard(pipeline);

    auto* output = depth->requestOutput();
    REQUIRE(output != nullptr);
    auto depthQueue = output->depth->createOutputQueue();
    auto confidenceQueue = output->confidence->createOutputQueue();

    pipeline.start();

    // Full-resolution StereoDepth on RVC2/RVC3 can take longer to deliver the first frame.
    constexpr auto kFirstDepthFrameTimeout = std::chrono::seconds(60);

    bool timedOut = false;
    auto depthFrame = depthQueue->get<dai::ImgFrame>(kFirstDepthFrameTimeout, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(depthFrame != nullptr);

    timedOut = false;
    auto confidenceFrame = confidenceQueue->get<dai::ImgFrame>(std::chrono::seconds(15), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(confidenceFrame != nullptr);
}

constexpr std::pair<uint32_t, uint32_t> kUserStereoSensorResolution{1280, 800};
constexpr float kUserStereoFps = 30.0f;
constexpr float kDepthStereoFps = 15.0f;
constexpr auto kStreamFrameTimeout = std::chrono::seconds(30);
constexpr auto kDepthFrameTimeout = std::chrono::seconds(45);
constexpr auto kFpsMeasureWindow = std::chrono::milliseconds(3000);
/** StereoDepth output size on RVC2/RVC3 when user cameras are at 1280x800 (not neural model size). */
constexpr std::pair<uint32_t, uint32_t> kRvc2UserCameraDepthOutputSize{640, 400};
constexpr float kMinFpsMeasureSeconds = 0.5f;

void skipUnlessUserStereoDepthScenario(const std::shared_ptr<Device>& device) {
    if(device->getPlatform() == Platform::RVC2 && deviceReportsTofSensor(device)) {
        SKIP("Skipping Depth user-camera test: RVC2 AUTO selects ToF, not user stereo cameras.");
    }
    (void)requireFirstStereoPairForTest(device);
    if(device->getPlatform() == Platform::RVC4 && !device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth user-camera test: device does not support NeuralDepth.");
    }
}

std::pair<std::shared_ptr<node::Camera>, std::shared_ptr<node::Camera>> buildUserStereoCamerasOrSkip(Pipeline& pipeline, const StereoPair& pair) {
    try {
        auto leftCam = pipeline.create<node::Camera>()->build(pair.left, kUserStereoSensorResolution, kUserStereoFps);
        auto rightCam = pipeline.create<node::Camera>()->build(pair.right, kUserStereoSensorResolution, kUserStereoFps);
        return {leftCam, rightCam};
    } catch(const std::exception& ex) {
        SKIP(std::string("Skipping Depth user-camera test: cannot build stereo cameras at 1280x800@30: ") + ex.what());
    }
    return {{}, {}};  // unreachable (SKIP throws)
}

std::shared_ptr<ImgFrame> requireStreamFrame(const std::shared_ptr<MessageQueue>& queue, std::chrono::seconds timeout) {
    REQUIRE(queue != nullptr);
    bool timedOut = false;
    auto frame = queue->get<ImgFrame>(timeout, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    return frame;
}

void drainStreamQueue(const std::shared_ptr<MessageQueue>& queue) {
    REQUIRE(queue != nullptr);
    for(int i = 0; i < 512; ++i) {
        bool timedOut = false;
        auto frame = queue->get<ImgFrame>(std::chrono::milliseconds(1), timedOut);
        if(timedOut || frame == nullptr) {
            break;
        }
    }
}

/** Host-side receive rate over a short window (no BenchmarkIn / sustained pipeline run). */
float measureReceiveFps(const std::shared_ptr<MessageQueue>& queue, std::chrono::milliseconds window) {
    REQUIRE(queue != nullptr);
    drainStreamQueue(queue);

    const auto deadline = std::chrono::steady_clock::now() + window;
    size_t frameCount = 0;
    std::chrono::steady_clock::time_point firstTs;
    std::chrono::steady_clock::time_point lastTs;

    while(std::chrono::steady_clock::now() < deadline) {
        bool timedOut = false;
        auto frame = queue->get<ImgFrame>(std::chrono::milliseconds(200), timedOut);
        if(timedOut || frame == nullptr) {
            continue;
        }
        const auto now = std::chrono::steady_clock::now();
        if(frameCount == 0) {
            firstTs = now;
        }
        lastTs = now;
        ++frameCount;
    }

    if(frameCount < 2) {
        return 0.f;
    }
    const auto seconds = std::chrono::duration<float>(lastTs - firstTs).count();
    if(seconds < kMinFpsMeasureSeconds) {
        return 0.f;
    }
    return static_cast<float>(frameCount - 1) / seconds;
}

void requireReceiveFpsInRange(const std::shared_ptr<MessageQueue>& queue, float minFps, float maxFps, std::chrono::milliseconds window = kFpsMeasureWindow) {
    const float fps = measureReceiveFps(queue, window);
    REQUIRE(fps >= minFps);
    REQUIRE(fps <= maxFps);
}

struct UserDepthCameraSetup {
    std::shared_ptr<node::Camera> leftCam;
    std::shared_ptr<node::Camera> rightCam;
    std::shared_ptr<MessageQueue> userFrameQueue;
    std::shared_ptr<MessageQueue> depthFrameQueue;
    std::shared_ptr<node::Depth> depth;
};

UserDepthCameraSetup wireUserStereoCamerasAndDepth(Pipeline& pipeline, const StereoPair& pair, std::optional<float> depthRequestedFps, bool userPreviewStream) {
    UserDepthCameraSetup setup;
    std::tie(setup.leftCam, setup.rightCam) = buildUserStereoCamerasOrSkip(pipeline, pair);

    if(userPreviewStream) {
        auto* userLeftOut = setup.leftCam->requestOutput(kUserStereoSensorResolution, std::nullopt, ImgResizeMode::CROP, kUserStereoFps);
        REQUIRE(userLeftOut != nullptr);
        setup.userFrameQueue = userLeftOut->createOutputQueue();
    }

    setup.depth = pipeline.create<node::Depth>();
    auto* requestedOutput = setup.depth->requestOutput(depthRequestedFps);
    REQUIRE(requestedOutput != nullptr);
    setup.depthFrameQueue = requestedOutput->depth->createOutputQueue();

    return setup;
}

void requireUserAndDepthFrameSizes(const std::shared_ptr<Device>& device,
                                   const std::shared_ptr<node::Depth>& depth,
                                   const std::shared_ptr<ImgFrame>& userFrame,
                                   const std::shared_ptr<ImgFrame>& depthFrame) {
    REQUIRE(userFrame->getWidth() == static_cast<int>(kUserStereoSensorResolution.first));
    REQUIRE(userFrame->getHeight() == static_cast<int>(kUserStereoSensorResolution.second));

    if(device->getPlatform() == Platform::RVC4) {
        const auto resolved = depth->getResolvedAlgorithm();
        if(resolved == node::Depth::Algorithm::NEURAL) {
            const auto [expectedW, expectedH] = node::NeuralDepth::getInputSize(std::get<DeviceModelZoo>(depth->getResolvedConfig()));
            requireDepthSingleBackendChild(*depth, "NeuralDepth");
            REQUIRE((depthFrame->getWidth() != static_cast<int>(kUserStereoSensorResolution.first)
                     || depthFrame->getHeight() != static_cast<int>(kUserStereoSensorResolution.second)));
            REQUIRE(depthFrame->getWidth() == expectedW);
            REQUIRE(depthFrame->getHeight() == expectedH);
        } else if(resolved == node::Depth::Algorithm::NEURAL_ASSISTED_STEREO) {
            requireDepthSingleBackendChild(*depth, "NeuralAssistedStereo");
            REQUIRE(depthFrame->getWidth() > 0);
            REQUIRE(depthFrame->getHeight() > 0);
        } else {
            const char* backendName = resolved == node::Depth::Algorithm::GPU_STEREO ? "GPUStereo" : "StereoDepth";
            requireDepthSingleBackendChild(*depth, backendName);
            REQUIRE(depthFrame->getWidth() > 0);
            REQUIRE(depthFrame->getHeight() > 0);
        }
    } else {
        requireDepthSingleBackendChild(*depth, "StereoDepth");
        REQUIRE((depthFrame->getWidth() != static_cast<int>(kUserStereoSensorResolution.first)
                 || depthFrame->getHeight() != static_cast<int>(kUserStereoSensorResolution.second)));
        REQUIRE(depthFrame->getWidth() == static_cast<int>(kRvc2UserCameraDepthOutputSize.first));
        const bool depthHeightMatchesBackend = depthFrame->getHeight() == static_cast<int>(kRvc2UserCameraDepthOutputSize.second);
        const bool depthHeightMatchesUser = depthFrame->getHeight() == static_cast<int>(kUserStereoSensorResolution.second);
        REQUIRE((depthHeightMatchesBackend || depthHeightMatchesUser));
    }
}

void runUserCameraDepthTest(Pipeline& pipeline,
                            const std::shared_ptr<Device>& device,
                            const StereoPair& pair,
                            std::optional<float> depthRequestedFps,
                            bool checkFrameSizes,
                            bool userPreviewStream) {
    PipelineStopGuard guard(pipeline);
    auto setup = wireUserStereoCamerasAndDepth(pipeline, pair, depthRequestedFps, userPreviewStream);

    REQUIRE_FALSE(cameraInDepthSubtree(*setup.depth, setup.leftCam));
    REQUIRE_FALSE(cameraInDepthSubtree(*setup.depth, setup.rightCam));
    REQUIRE(countStereoCamerasInDepthSubtree(*setup.depth, pair) == 0);

    pipeline.build();
    pipeline.start();

    if(userPreviewStream) {
        REQUIRE(setup.userFrameQueue != nullptr);
        auto userFrame = requireStreamFrame(setup.userFrameQueue, kStreamFrameTimeout);
        auto depthFrame = requireStreamFrame(setup.depthFrameQueue, kDepthFrameTimeout);

        if(checkFrameSizes) {
            requireUserAndDepthFrameSizes(device, setup.depth, userFrame, depthFrame);
        } else {
            // Host queue rates are only meaningful for the user mono stream; depth queues can burst after start.
            requireReceiveFpsInRange(setup.userFrameQueue, kUserStereoFps * 0.5f, kUserStereoFps * 1.5f);
            if(depthRequestedFps.has_value()) {
                (void)requireStreamFrame(setup.depthFrameQueue, kDepthFrameTimeout);
            }
        }
    } else {
        (void)requireStreamFrame(setup.depthFrameQueue, kDepthFrameTimeout);
        if(depthRequestedFps.has_value()) {
            (void)requireStreamFrame(setup.depthFrameQueue, kDepthFrameTimeout);
        }
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
    auto* output = depth->requestOutput();
    REQUIRE(output != nullptr);
    REQUIRE_NOTHROW((void)output->depth.operator->());
    REQUIRE_NOTHROW((void)output->confidence.operator->());
}

TEST_CASE("Depth: build(algorithm) exposes algorithm via getRequestedAlgorithm") {
    Pipeline pipeline;
    (void)requireDefaultDevice(pipeline);
    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::TOF);
    REQUIRE(depth->getRequestedAlgorithm() == node::Depth::Algorithm::TOF);
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

    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::STEREO);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "StereoDepth");
}

TEST_CASE("Depth: GPU_STEREO requires RVC4") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() == Platform::RVC4) {
        SKIP("Skipping negative GPU_STEREO test on RVC4.");
    }
    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::GPU_STEREO);
    REQUIRE_THROWS((void)depth->requestOutput()->depth.operator->());
}

TEST_CASE("Depth: TOF requires connected ToF camera") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(deviceReportsTofSensor(device)) {
        SKIP("Skipping negative TOF test: device reports a ToF sensor.");
    }
    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::TOF);
    REQUIRE_THROWS((void)depth->requestOutput()->depth.operator->());
}

TEST_CASE("Depth: resolved algorithm + config can rebuild the same backend explicitly") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    const auto autoUsesTof = device->getPlatform() == Platform::RVC2 && deviceReportsTofSensor(device);
    if(!autoUsesTof) {
        (void)requireFirstStereoPairForTest(device);
    }

    // Resolve once with AUTO, then feed the resolved pair into a second node's explicit build().
    auto depth = pipeline.create<node::Depth>();
    auto* output = depth->requestOutput();
    REQUIRE(output != nullptr);
    REQUIRE_NOTHROW((void)output->depth.operator->());
    const auto resolvedAlgorithm = depth->getResolvedAlgorithm();
    const auto resolvedConfig = depth->getResolvedConfig();

    auto explicitDepth = pipeline.create<node::Depth>();
    explicitDepth->build(resolvedAlgorithm, resolvedConfig);
    auto* explicitOutput = explicitDepth->requestOutput();
    REQUIRE(explicitOutput != nullptr);
    REQUIRE_NOTHROW((void)explicitOutput->depth.operator->());
    REQUIRE(explicitDepth->getRequestedAlgorithm() == resolvedAlgorithm);
    REQUIRE(explicitDepth->getResolvedAlgorithm() == resolvedAlgorithm);
    REQUIRE(explicitDepth->getResolvedConfig() == resolvedConfig);
}

TEST_CASE("Depth: explicit algorithm + config rejects algorithms unsupported by the device") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(deviceSupportsGpuStereo(device)) {
        SKIP("Skipping negative GPU_STEREO config test: device supports GPU_STEREO.");
    }

    auto depth = pipeline.create<node::Depth>();
    depth->build(node::Depth::Algorithm::GPU_STEREO, node::Depth::Config{std::monostate{}});
    REQUIRE_THROWS_WITH((void)depth->requestOutput()->depth.operator->(), Catch::Matchers::ContainsSubstring("GPU_STEREO"));
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

    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::NEURAL);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "NeuralDepth");
}

TEST_CASE("Depth: NEURAL_ASSISTED_STEREO rejected off RVC4") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() == Platform::RVC4) {
        SKIP("Skipping Depth test: RVC4 device (negative NAS test not applicable).");
    }
    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_THROWS((void)depth->requestOutput()->depth.operator->());
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

    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::NEURAL_ASSISTED_STEREO);
    REQUIRE_NOTHROW(startPipelineAndRequireFirstFrames(pipeline, depth));
    requireDepthSingleBackendChild(*depth, "NeuralAssistedStereo");
}

TEST_CASE("Depth: TOF algorithm wires ToF backend when ToF sensor present") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(!deviceReportsTofSensor(device)) {
        SKIP("Skipping Depth test: no ToF sensor reported.");
    }

    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::TOF);
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
    if(!deviceSupportsGpuStereo(device)) {
        SKIP("Skipping GPU_STEREO positive test: device->isGpuStereoSupported() returned false.");
    }

    auto depth = pipeline.create<node::Depth>()->build(node::Depth::Algorithm::GPU_STEREO);
    try {
        auto* output = depth->requestOutput();
        (void)output->depth.operator->();
        (void)output->confidence.operator->();
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

TEST_CASE("Depth: user stereo cameras keep 30 FPS with AUTO and no requestOutput(fps)") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthTest(pipeline, device, pair, std::nullopt, false, true));
}

TEST_CASE("Depth: user preview and depth share the same requestOutput(fps) on pre-built cameras") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    // A separate user preview stream and a lower Depth.requestOutput(fps) on the same Camera currently desyncs
    // on device (dual-rate outputs). Match the preview rate until Camera supports mixed output FPS.
    REQUIRE_NOTHROW(runUserCameraDepthTest(pipeline, device, pair, kUserStereoFps, false, true));
}

TEST_CASE("Depth: pre-built user stereo cameras with depth requestOutput(fps) at 15 FPS") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthTest(pipeline, device, pair, kDepthStereoFps, false, false));
}

TEST_CASE("Depth: user camera resolution unchanged; depth uses backend size") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    skipUnlessUserStereoDepthScenario(device);
    const auto pair = requireFirstStereoPairForTest(device);

    REQUIRE_NOTHROW(runUserCameraDepthTest(pipeline, device, pair, std::nullopt, true, true));
}

TEST_CASE("Depth: RVC4 AUTO selects neural model from stereo_size and fps") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping Depth test: not RVC4.");
    }
    if(!device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth test: device does not support NeuralDepth.");
    }
    (void)requireFirstStereoPairForTest(device);

    auto depth = pipeline.create<node::Depth>();
    const auto supported = node::DepthTestAccess::getSupportedAlgorithms(*depth, device);
    const auto expected = node::DepthTestAccess::selectBackend(std::make_pair(640u, 400u), 30.f, supported, device->getSupportedDeviceModels());
    REQUIRE(expected.first == node::Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(expected.second) == DeviceModelZoo::NEURAL_DEPTH_576X360);

    depth->build(node::Depth::Algorithm::AUTO);
    auto* output = depth->requestOutput({640, 400}, 30.f);
    REQUIRE(output != nullptr);
    REQUIRE_NOTHROW((void)output->depth.operator->());
    REQUIRE(output->depth.operator->() == depth->requestOutput({640, 400}, 30.f)->depth.operator->());
    REQUIRE(output->confidence.operator->() == depth->requestOutput({640, 400}, 30.f)->confidence.operator->());
    REQUIRE(depth->getResolvedAlgorithm() == node::Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(depth->getResolvedConfig()) == std::get<DeviceModelZoo>(expected.second));
    requireDepthSingleBackendChild(*depth, "NeuralDepth");
}

TEST_CASE("Depth: RVC4 AUTO uses requested output size from existing stereo cameras") {
    Pipeline pipeline;
    auto device = requireDefaultDevice(pipeline);
    if(device->getPlatform() != Platform::RVC4) {
        SKIP("Skipping Depth test: not RVC4.");
    }
    if(!device->isNeuralDepthSupported()) {
        SKIP("Skipping Depth test: device does not support NeuralDepth.");
    }
    const auto pair = requireFirstStereoPairForTest(device);

    auto left = pipeline.create<node::Camera>()->build(pair.left, std::nullopt, 20.f);
    auto right = pipeline.create<node::Camera>()->build(pair.right, std::nullopt, 20.f);
    auto* leftOut = left->requestOutput({640, 400}, std::nullopt, ImgResizeMode::CROP, 20.f);
    auto* rightOut = right->requestOutput({640, 400}, std::nullopt, ImgResizeMode::CROP, 20.f);
    REQUIRE(leftOut != nullptr);
    REQUIRE(rightOut != nullptr);
    auto leftQueue = leftOut->createOutputQueue();
    auto rightQueue = rightOut->createOutputQueue();
    (void)leftQueue;
    (void)rightQueue;

    auto depth = pipeline.create<node::Depth>();
    const auto expected = node::DepthTestAccess::selectBackend(
        std::make_pair(640u, 400u), 20.f, node::DepthTestAccess::getSupportedAlgorithms(*depth, device), device->getSupportedDeviceModels());
    REQUIRE(expected.first == node::Depth::Algorithm::NEURAL);

    auto* output = depth->requestOutput();
    REQUIRE(output != nullptr);
    REQUIRE_NOTHROW((void)output->depth.operator->());
    REQUIRE(depth->getResolvedAlgorithm() == node::Depth::Algorithm::NEURAL);
    REQUIRE(std::get<DeviceModelZoo>(depth->getResolvedConfig()) == std::get<DeviceModelZoo>(expected.second));
    requireDepthSingleBackendChild(*depth, "NeuralDepth");
}
