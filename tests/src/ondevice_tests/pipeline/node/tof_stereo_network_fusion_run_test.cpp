// Behavioural on-device tests of the ToFStereoFusion STEREO_NETWORK model: depth frames must arrive, from the requested
// backend, with the rectified geometry the networks are trained on, and keep arriving when the ToF leaves the fusion depth
// mode.
//
// The network is not in the model zoo, so the tests need it from the environment and skip without it:
//   DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL  path of the network (.dlc) on the host
//   DEPTHAI_TEST_TOF_STEREO_NETWORK_UDO    registration library of its custom operators on the device (optional)
//   DEPTHAI_TEST_TOF_STEREO_NETWORK_SIZE   network input size, e.g. 1280x800 (default) or 1024x640
#include <algorithm>
#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "depthai/beta/node/ToFStereoFusion.hpp"
#include "depthai/depthai.hpp"

using namespace dai;
using Fusion = beta::node::ToFStereoFusion;
using FusionBackend = beta::ToFStereoFusionProperties::Backend;

namespace {

constexpr auto FIRST_FRAME_TIMEOUT = std::chrono::seconds(30);  // network load included
constexpr auto RUN_TIME = std::chrono::seconds(10);
constexpr size_t MIN_FRAMES = 60;  // of ~300 at 30 fps; the CPU backend reaches ~24 fps at 1280x800

bool deviceHasTofSensor(const std::shared_ptr<Device>& device) {
    for(const auto& features : device->getConnectedCameraFeatures()) {
        for(const auto type : features.supportedTypes) {
            if(type == CameraSensorType::TOF) return true;
        }
    }
    return false;
}

struct PipelineStopGuard {
    Pipeline& pipeline;
    explicit PipelineStopGuard(Pipeline& p) : pipeline(p) {}
    ~PipelineStopGuard() {
        if(pipeline.isRunning()) pipeline.stop();
    }
    PipelineStopGuard(const PipelineStopGuard&) = delete;
    PipelineStopGuard& operator=(const PipelineStopGuard&) = delete;
};

struct ModelSetup {
    std::string model;
    std::string udo;
    uint32_t width = 1280;
    uint32_t height = 800;
};

std::optional<ModelSetup> modelFromEnvironment() {
    const char* model = std::getenv("DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL");
    if(!model || !*model) return std::nullopt;
    ModelSetup setup;
    setup.model = model;
    if(const char* udo = std::getenv("DEPTHAI_TEST_TOF_STEREO_NETWORK_UDO")) setup.udo = udo;
    if(const char* size = std::getenv("DEPTHAI_TEST_TOF_STEREO_NETWORK_SIZE")) {
        unsigned width = 0, height = 0;
        if(std::sscanf(size, "%ux%u", &width, &height) == 2) {
            setup.width = width;
            setup.height = height;
        }
    }
    return setup;
}

/** Device log lines of the fusion node, collected from the device log callback. */
class LogCollector {
   public:
    void add(const LogMessage& message) {
        std::lock_guard<std::mutex> lock(mutex);
        lines.push_back(message.payload);
    }
    bool contains(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex);
        for(const auto& line : lines)
            if(line.find(text) != std::string::npos) return true;
        return false;
    }

   private:
    std::mutex mutex;
    std::vector<std::string> lines;
};

std::shared_ptr<Fusion> buildFusion(Pipeline& pipeline, const ModelSetup& setup, FusionBackend backend) {
    auto left = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto right = pipeline.create<node::Camera>()->build(CameraBoardSocket::CAM_C, std::nullopt, 30.0f);
    auto fusion = pipeline.create<Fusion>();
    fusion->setModel(Fusion::Properties::Model::STEREO_NETWORK).setNetworkInputSize(setup.width, setup.height).setBackend(backend);
    fusion->build(left, right);
    fusion->neuralNetwork->setModelPath(setup.model);
    std::map<std::string, std::string> backendProperties{{"output_tensors", "disp_low,head9,conf"}, {"output_memory", "dma_buf"}, {"register_dma_buf", "true"}};
    if(!setup.udo.empty()) backendProperties["udo_package"] = setup.udo;
    fusion->neuralNetwork->setBackendProperties(backendProperties);
    return fusion;
}

/** fx of cv::stereoRectify(alpha 0, CALIB_ZERO_DISPARITY) for the left/right pair at the given size. */
float expectedRectifiedFx(const CalibrationHandler& calibration, uint32_t width, uint32_t height) {
    const auto toMat = [](const std::vector<std::vector<float>>& rows) {
        cv::Mat mat(static_cast<int>(rows.size()), static_cast<int>(rows[0].size()), CV_64F);
        for(size_t r = 0; r < rows.size(); ++r)
            for(size_t c = 0; c < rows[r].size(); ++c) mat.at<double>(static_cast<int>(r), static_cast<int>(c)) = rows[r][c];
        return mat;
    };
    const auto distortion = [](const std::vector<float>& values) {
        cv::Mat mat(1, static_cast<int>(values.size()), CV_64F);
        for(size_t i = 0; i < values.size(); ++i) mat.at<double>(0, static_cast<int>(i)) = values[i];
        return mat;
    };
    const cv::Mat k1 = toMat(calibration.getCameraIntrinsics(CameraBoardSocket::CAM_B, static_cast<int>(width), static_cast<int>(height)));
    const cv::Mat k2 = toMat(calibration.getCameraIntrinsics(CameraBoardSocket::CAM_C, static_cast<int>(width), static_cast<int>(height)));
    const cv::Mat d1 = distortion(calibration.getDistortionCoefficients(CameraBoardSocket::CAM_B));
    const cv::Mat d2 = distortion(calibration.getDistortionCoefficients(CameraBoardSocket::CAM_C));
    const cv::Mat extrinsics = toMat(calibration.getCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C));
    const cv::Mat rotation = extrinsics(cv::Rect(0, 0, 3, 3)).clone();
    const cv::Mat translation = extrinsics(cv::Rect(3, 0, 1, 3)).clone();
    const cv::Size size(static_cast<int>(width), static_cast<int>(height));
    cv::Mat r1, r2, p1, p2, q;
    cv::stereoRectify(k1, d1, k2, d2, size, rotation, translation, r1, r2, p1, p2, q, cv::CALIB_ZERO_DISPARITY, 0.0, size);
    return static_cast<float>(p1.at<double>(0, 0));
}

}  // namespace

TEST_CASE("ToFStereoFusion STEREO_NETWORK outputs depth from the GPU backend with the trained rectification", "[ToFStereoFusion][behaviour]") {
    const auto setup = modelFromEnvironment();
    if(!setup) SKIP("set DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL (and _UDO, _SIZE) to the fusion network");
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");
    if(!device->hasGPU()) SKIP("needs a device with a GPU");

    LogCollector logs;
    device->setLogLevel(LogLevel::INFO);
    device->addLogCallback([&logs](LogMessage message) { logs.add(message); });

    Pipeline pipeline(device);
    auto fusion = buildFusion(pipeline, *setup, FusionBackend::GPU);
    auto depthQueue = fusion->depth.createOutputQueue(8, false);

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    bool timedOut = false;
    const auto first = depthQueue->get<ImgFrame>(FIRST_FRAME_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(first != nullptr);
    REQUIRE(first->getType() == ImgFrame::Type::RAW16);
    REQUIRE(first->getWidth() == setup->width);
    REQUIRE(first->getHeight() == setup->height);

    size_t frames = 0, validPixels = 0, pixels = 0;
    std::vector<double> frameValid;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < RUN_TIME) {
        const auto depth = depthQueue->get<ImgFrame>(std::chrono::seconds(2), timedOut);
        REQUIRE_FALSE(timedOut);  // no output gap longer than 2 s
        ++frames;
        const auto* data = reinterpret_cast<const uint16_t*>(depth->getData().data());
        const size_t count = static_cast<size_t>(depth->getWidth()) * depth->getHeight();
        size_t frameValidPixels = 0, framePixels = 0;
        for(size_t i = 0; i < count; i += 97) {
            frameValidPixels += data[i] != 0 ? 1 : 0;
            ++framePixels;
        }
        validPixels += frameValidPixels;
        pixels += framePixels;
        frameValid.push_back(static_cast<double>(frameValidPixels) / static_cast<double>(framePixels));
    }
    INFO("depth frames in " << RUN_TIME.count() << " s: " << frames << ", valid " << (100.0 * validPixels / pixels) << " %");
    REQUIRE(frames >= MIN_FRAMES);
    REQUIRE(validPixels * 10 >= pixels);  // at least 10 % of the pixels have depth
    // No partly broken frames (static scene: every frame close to the typical one).
    std::vector<double> sorted = frameValid;
    std::sort(sorted.begin(), sorted.end());
    const double medianValid = sorted[sorted.size() / 2];
    INFO("valid fraction per frame: min " << sorted.front() << ", median " << medianValid);
    REQUIRE(sorted.front() >= 0.7 * medianValid);

    // The GPU backend was selected and did not fall back.
    REQUIRE(logs.contains("STEREO_NETWORK backend GPU"));
    REQUIRE_FALSE(logs.contains("GPU backend unavailable"));
    REQUIRE_FALSE(logs.contains("unexpected network output layout"));

    // Rectified with alpha 0 (the training geometry), not with the left input intrinsics.
    const auto intrinsics = first->transformation.getIntrinsicMatrix();
    const float expectedFx = expectedRectifiedFx(device->readCalibration(), setup->width, setup->height);
    INFO("depth fx " << intrinsics[0][0] << ", expected (alpha 0) " << expectedFx);
    REQUIRE(std::abs(intrinsics[0][0] - expectedFx) < 0.01f * expectedFx);
}

TEST_CASE("ToFStereoFusion STEREO_NETWORK keeps producing depth when the ToF leaves the fusion depth mode", "[ToFStereoFusion][behaviour]") {
    const auto setup = modelFromEnvironment();
    if(!setup) SKIP("set DEPTHAI_TEST_TOF_STEREO_NETWORK_MODEL (and _UDO, _SIZE) to the fusion network");
    const auto device = std::make_shared<Device>();
    if(device->getPlatform() != Platform::RVC4 || !deviceHasTofSensor(device)) SKIP("needs an RVC4 device with a ToF sensor");

    LogCollector logs;
    device->setLogLevel(LogLevel::WARN);
    device->addLogCallback([&logs](LogMessage message) { logs.add(message); });

    Pipeline pipeline(device);
    auto fusion = buildFusion(pipeline, *setup, FusionBackend::AUTO);
    auto depthQueue = fusion->depth.createOutputQueue(8, false);
    auto tofConfigQueue = fusion->tof->tofBaseNode.inputConfig.createInputQueue();

    PipelineStopGuard guard(pipeline);
    REQUIRE_NOTHROW(pipeline.start());

    bool timedOut = false;
    REQUIRE(depthQueue->get<ImgFrame>(FIRST_FRAME_TIMEOUT, timedOut) != nullptr);
    REQUIRE_FALSE(timedOut);

    // The vendor unwrap: the same state as the fusion mode falling back on a device without a GPU.
    auto standard = std::make_shared<ToFConfig>(*fusion->tof->tofBaseNode.initialConfig);
    standard->setDepthMode(ToFConfig::DepthMode::STANDARD);
    tofConfigQueue->send(standard);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    depthQueue->tryGetAll<ImgFrame>();

    size_t frames = 0;
    const auto start = std::chrono::steady_clock::now();
    while(std::chrono::steady_clock::now() - start < RUN_TIME) {
        const auto depth = depthQueue->get<ImgFrame>(std::chrono::seconds(2), timedOut);
        INFO("depth frames after the switch to STANDARD: " << frames);
        REQUIRE_FALSE(timedOut);  // a stall here is the phase/Sync starvation
        ++frames;
    }
    REQUIRE(frames >= MIN_FRAMES);
    REQUIRE(logs.contains("vendor-unwrap frames without phases"));

    // Back to the fusion depth mode.
    auto fusionMode = std::make_shared<ToFConfig>(*standard);
    fusionMode->setDepthMode(ToFConfig::DepthMode::FUSION);
    tofConfigQueue->send(fusionMode);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    depthQueue->tryGetAll<ImgFrame>();
    REQUIRE(depthQueue->get<ImgFrame>(std::chrono::seconds(5), timedOut) != nullptr);
    REQUIRE_FALSE(timedOut);
}
