#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

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
