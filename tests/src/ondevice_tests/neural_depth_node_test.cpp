#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <magic_enum/magic_enum.hpp>

#include "depthai/depthai.hpp"

// Disable container overflow detection for this test binary (false positive from protobuf)
extern "C" const char* __asan_default_options() {
    return "detect_container_overflow=0";
}

namespace {
constexpr std::array<dai::DeviceModelZoo, 10> kNeuralDepthModels = {
    dai::DeviceModelZoo::NEURAL_DEPTH_1248X780,
    dai::DeviceModelZoo::NEURAL_DEPTH_1056X660,
    dai::DeviceModelZoo::NEURAL_DEPTH_960X600,
    dai::DeviceModelZoo::NEURAL_DEPTH_864X540,
    dai::DeviceModelZoo::NEURAL_DEPTH_768X480,
    dai::DeviceModelZoo::NEURAL_DEPTH_576X360,
    dai::DeviceModelZoo::NEURAL_DEPTH_480X300,
    dai::DeviceModelZoo::NEURAL_DEPTH_384X240,
    dai::DeviceModelZoo::NEURAL_DEPTH_288X180,
    dai::DeviceModelZoo::NEURAL_DEPTH_192X120,
};

struct LiveCameraTestCase {
    dai::DeviceModelZoo model;
    float minFps;
};

constexpr std::array<LiveCameraTestCase, kNeuralDepthModels.size()> kLiveCameraTestCases = {{
    {dai::DeviceModelZoo::NEURAL_DEPTH_1248X780, 1.6f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_1056X660, 3.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_960X600, 5.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_864X540, 8.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_768X480, 10.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_576X360, 24.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_480X300, 40.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_384X240, 55.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_288X180, 55.0f},
    {dai::DeviceModelZoo::NEURAL_DEPTH_192X120, 55.0f},
}};

bool isModelSupported(const std::shared_ptr<dai::DeviceBase>& device, dai::DeviceModelZoo model) {
    const auto supportedModels = device->getSupportedDeviceModels();
    return std::find(supportedModels.begin(), supportedModels.end(), model) != supportedModels.end();
}

void testNeuralDepthModelBasic(dai::DeviceModelZoo model, float minFps) {
    constexpr float kFpsTolerance = 0.2f;

    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(!isModelSupported(device, model)) {
        WARN("Skipping NeuralDepth live-camera test: model " << magic_enum::enum_name(model) << " is not supported on this device.");
        return;
    }
    constexpr float FPS = 60.0f;
    auto leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto leftOutput = leftCamera->requestFullResolutionOutput(std::nullopt, FPS);
    auto rightOutput = rightCamera->requestFullResolutionOutput(std::nullopt, FPS);

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    neuralDepth->build(*leftOutput, *rightOutput, model);

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(2 * minFps);
    neuralDepth->depth.link(benchmarkIn->input);

    auto benchmarkOutputQueue = benchmarkIn->report.createOutputQueue(15, false);
    auto disparityQueue = neuralDepth->disparity.createOutputQueue();
    auto depthQueue = neuralDepth->depth.createOutputQueue();
    auto edgeQueue = neuralDepth->edge.createOutputQueue();
    auto confidenceQueue = neuralDepth->confidence.createOutputQueue();

    pipeline.start();

    constexpr size_t NUM_FRAMES = 10;
    for(size_t i = 0; i < NUM_FRAMES; ++i) {
        REQUIRE(disparityQueue->get<dai::ImgFrame>() != nullptr);
        REQUIRE(depthQueue->get<dai::ImgFrame>() != nullptr);
        REQUIRE(edgeQueue->get<dai::ImgFrame>() != nullptr);
        REQUIRE(confidenceQueue->get<dai::ImgFrame>() != nullptr);
    }

    for(int i = 0; i < 5; ++i) {
        auto report = benchmarkOutputQueue->get<dai::BenchmarkReport>();
        if(i == 0) {
            continue;
        }
        REQUIRE(report->fps + kFpsTolerance >= minFps);
    }

    pipeline.stop();
}
}  // namespace

constexpr size_t FRAMES_TO_SAMPLE = 12;

struct DepthStats {
    double median = 0.0;
    double validRatio = 0.0;
};

DepthStats computeDepthStats(const std::shared_ptr<dai::ImgFrame>& depthFrame) {
    DepthStats stats{};
    cv::Mat depthMat = depthFrame->getFrame();
    if(depthMat.empty()) {
        return stats;
    }

    const size_t totalPixels = depthMat.total();
    std::vector<double> samples;
    samples.reserve(totalPixels);

    const auto* ptr = depthMat.ptr<uint16_t>();
    for(size_t i = 0; i < totalPixels; ++i) {
        double val = static_cast<double>(ptr[i]);
        if(val > 0.0) {
            samples.push_back(val);
        }
    }

    stats.validRatio = samples.empty() ? 0.0 : static_cast<double>(samples.size()) / static_cast<double>(totalPixels);
    if(samples.empty()) {
        return stats;
    }

    auto mid = samples.size() / 2;
    std::nth_element(samples.begin(), samples.begin() + mid, samples.end());
    stats.median = samples[mid];
    return stats;
}

TEST_CASE("NeuralDepth replay produces expected results") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();

    pipeline.enableHolisticReplay(NEURAL_REPLAY_PATH);
    pipeline.setCalibrationData(dai::CalibrationHandler(NEURAL_CALIBRATION_PATH));

    auto cameraLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto leftOutput = cameraLeft->requestFullResolutionOutput();
    auto rightOutput = cameraRight->requestFullResolutionOutput();

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    const auto& modelToRun = GENERATE_REF(from_range(kNeuralDepthModels));
    INFO("Model: " << magic_enum::enum_name(modelToRun));
    if(!isModelSupported(device, modelToRun)) {
        WARN("Skipping NeuralDepth replay test: model " << magic_enum::enum_name(modelToRun) << " is not supported on this device.");
        return;
    }
    neuralDepth->build(*leftOutput, *rightOutput, modelToRun);
    neuralDepth->left.setBlocking(true);
    neuralDepth->right.setBlocking(true);
    auto depthQueue = neuralDepth->depth.createOutputQueue();
    auto disparityQueue = neuralDepth->disparity.createOutputQueue();
    auto confidenceQueue = neuralDepth->confidence.createOutputQueue();

    pipeline.start();
    auto sequenceNumber = 0;
    for(size_t idx = 0; idx < FRAMES_TO_SAMPLE; ++idx) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        auto confidence = confidenceQueue->get<dai::ImgFrame>();
        auto depth = depthQueue->get<dai::ImgFrame>();
        if(idx == 0) {
            sequenceNumber = depth->getSequenceNum();
            continue;  // Skip the first frame as it can have encoding artifacts
        }
        REQUIRE(disparity != nullptr);
        REQUIRE(confidence != nullptr);
        REQUIRE(depth != nullptr);

        auto stats = computeDepthStats(depth);
        INFO("Frame " << idx << " valid ratio: " << stats.validRatio);
        REQUIRE(stats.validRatio > 0.90);
        REQUIRE(stats.validRatio < 1.0);

        INFO("NeuralDepth median = " << stats.median);
        REQUIRE(stats.median > 720);
        REQUIRE(stats.median < 790);

        REQUIRE(depth->getSequenceNum() == sequenceNumber + 1);
        sequenceNumber++;
    }

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("NeuralDepth replay aligns with StereoDepth medians") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();

    pipeline.enableHolisticReplay(NEURAL_REPLAY_PATH);
    pipeline.setCalibrationData(dai::CalibrationHandler(NEURAL_CALIBRATION_PATH));

    auto cameraLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto leftOutput = cameraLeft->requestFullResolutionOutput();
    auto rightOutput = cameraRight->requestFullResolutionOutput();

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    const auto& modelToRun = GENERATE_REF(from_range(kNeuralDepthModels));
    INFO("Model: " << magic_enum::enum_name(modelToRun));
    if(!isModelSupported(device, modelToRun)) {
        WARN("Skipping NeuralDepth replay comparison test: model " << magic_enum::enum_name(modelToRun) << " is not supported on this device.");
        return;
    }
    neuralDepth->build(*leftOutput, *rightOutput, modelToRun);

    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();
    stereoDepth->build(*leftOutput, *rightOutput);
    stereoDepth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::ACCURACY);

    auto neuralDepthQueue = neuralDepth->depth.createOutputQueue();
    auto stereoDepthQueue = stereoDepth->depth.createOutputQueue();

    pipeline.start();

    for(size_t idx = 0; idx < FRAMES_TO_SAMPLE; ++idx) {
        auto neuralDepthFrame = neuralDepthQueue->get<dai::ImgFrame>();
        auto stereoDepthFrame = stereoDepthQueue->get<dai::ImgFrame>();
        REQUIRE(neuralDepthFrame != nullptr);
        REQUIRE(stereoDepthFrame != nullptr);

        auto neuralStats = computeDepthStats(neuralDepthFrame);
        auto stereoStats = computeDepthStats(stereoDepthFrame);
        REQUIRE(neuralStats.median > (0.9 * stereoStats.median));
        REQUIRE(neuralStats.median < (1.1 * stereoStats.median));
    }
}

TEST_CASE("Test NeuralDepth node live-camera models") {
    const auto& testCase = GENERATE_REF(from_range(kLiveCameraTestCases));
    INFO("Model: " << magic_enum::enum_name(testCase.model));
    testNeuralDepthModelBasic(testCase.model, testCase.minFps);
}
