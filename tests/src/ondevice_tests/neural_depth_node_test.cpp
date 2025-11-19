#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include "depthai/depthai.hpp"

namespace {
void testNeuralDepthModelBasic(dai::DeviceModelZoo model, float minFps) {
    dai::Pipeline pipeline;
    constexpr float FPS = 60.0f;
    auto leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto leftOutput = leftCamera->requestFullResolutionOutput(std::nullopt, FPS);
    auto rightOutput = rightCamera->requestFullResolutionOutput(std::nullopt, FPS);

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    neuralDepth->build(*leftOutput, *rightOutput, model);

    auto benchmarkIn = pipeline.create<dai::node::BenchmarkIn>();
    benchmarkIn->sendReportEveryNMessages(10);
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
        REQUIRE(report->fps >= minFps);
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

    const int matType = depthMat.type();
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
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping NeuralDepth replay test: device doesn't support NeuralDepth.");
        return;
    }

    pipeline.enableHolisticReplay(NEURAL_REPLAY_PATH);
    pipeline.setCalibrationData(dai::CalibrationHandler(NEURAL_CALIBRATION_PATH));

    auto cameraLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto leftOutput = cameraLeft->requestFullResolutionOutput();
    auto rightOutput = cameraRight->requestFullResolutionOutput();

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    SECTION("Test LARGE model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);
    }
    SECTION("Test MEDIUM model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM);
    }
    SECTION("Test SMALL model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_SMALL);
    }
    SECTION("Test NANO model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_NANO);
    }

    auto depthQueue = neuralDepth->depth.createOutputQueue();
    auto disparityQueue = neuralDepth->disparity.createOutputQueue();
    auto confidenceQueue = neuralDepth->confidence.createOutputQueue();

    pipeline.start();

    for(size_t idx = 0; idx < FRAMES_TO_SAMPLE; ++idx) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        auto confidence = confidenceQueue->get<dai::ImgFrame>();
        auto depth = depthQueue->get<dai::ImgFrame>();
        REQUIRE(disparity != nullptr);
        REQUIRE(confidence != nullptr);
        REQUIRE(depth != nullptr);

        auto stats = computeDepthStats(depth);
        INFO("Frame " << idx << " valid ratio: " << stats.validRatio);
        REQUIRE(stats.validRatio > 0.95);
        REQUIRE(stats.validRatio < 1.0);

        INFO("NeuralDepth median = " << stats.median);
        REQUIRE(stats.median > 720);
        REQUIRE(stats.median < 790);
    }

    pipeline.stop();
    pipeline.wait();
}

TEST_CASE("NeuralDepth replay aligns with StereoDepth medians") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    if(!device->isNeuralDepthSupported()) {
        WARN("Skipping NeuralDepth replay comparison test: device doesn't support NeuralDepth.");
        return;
    }

    pipeline.enableHolisticReplay(NEURAL_REPLAY_PATH);
    pipeline.setCalibrationData(dai::CalibrationHandler(NEURAL_CALIBRATION_PATH));

    auto cameraLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto cameraRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto leftOutput = cameraLeft->requestFullResolutionOutput();
    auto rightOutput = cameraRight->requestFullResolutionOutput();

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    SECTION("Test NANO model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_NANO);
    }
    SECTION("Test SMALL model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_SMALL);
    }
    SECTION("Test MEDIUM model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM);
    }
    SECTION("Test LARGE model") {
        neuralDepth->build(*leftOutput, *rightOutput, dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);
    }

    auto stereoDepth = pipeline.create<dai::node::StereoDepth>();
    stereoDepth->build(*leftOutput, *rightOutput);
    stereoDepth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::FAST_ACCURACY);

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

TEST_CASE("Test NeuralDepth node NANO model") {
    testNeuralDepthModelBasic(dai::DeviceModelZoo::NEURAL_DEPTH_NANO, 55.0f);
}

TEST_CASE("Test NeuralDepth node SMALL model") {
    testNeuralDepthModelBasic(dai::DeviceModelZoo::NEURAL_DEPTH_SMALL, 40.0f);
}

TEST_CASE("Test NeuralDepth node MEDIUM model") {
    testNeuralDepthModelBasic(dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM, 24.0f);
}

TEST_CASE("Test NeuralDepth node LARGE model") {
    testNeuralDepthModelBasic(dai::DeviceModelZoo::NEURAL_DEPTH_LARGE, 10.0f);
}
