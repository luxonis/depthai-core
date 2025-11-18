#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

namespace {
void testNeuralDepthModel(dai::DeviceModelZoo model) {
    dai::Pipeline pipeline;

    auto leftCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto rightCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto leftOutput = leftCamera->requestFullResolutionOutput();
    auto rightOutput = rightCamera->requestFullResolutionOutput();

    auto neuralDepth = pipeline.create<dai::node::NeuralDepth>();
    neuralDepth->build(*leftOutput, *rightOutput, model);

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

    pipeline.stop();
}
}  // namespace


TEST_CASE("Test NeuralDepth node NANO model") {
    testNeuralDepthModel(dai::DeviceModelZoo::NEURAL_DEPTH_NANO);
}

TEST_CASE("Test NeuralDepth node SMALL model") {
    testNeuralDepthModel(dai::DeviceModelZoo::NEURAL_DEPTH_SMALL);
}

TEST_CASE("Test NeuralDepth node MEDIUM model") {
    testNeuralDepthModel(dai::DeviceModelZoo::NEURAL_DEPTH_MEDIUM);
}

TEST_CASE("Test NeuralDepth node LARGE model") {
    testNeuralDepthModel(dai::DeviceModelZoo::NEURAL_DEPTH_LARGE);
}
