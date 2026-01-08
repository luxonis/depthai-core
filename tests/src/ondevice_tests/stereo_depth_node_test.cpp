#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

void testStereoDepthPreset(dai::node::StereoDepth::PresetMode preset, dai::ProcessorType backend = dai::ProcessorType::CPU) {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = p.create<dai::node::StereoDepth>()->build(*left->requestOutput(std::make_pair(640, 400)), *right->requestOutput(std::make_pair(640, 400)));

    stereo->setDefaultProfilePreset(preset);
    stereo->initialConfig->setFiltersComputeBackend(backend);

    auto disparityQueue = stereo->disparity.createOutputQueue();
    auto depthQueue = stereo->depth.createOutputQueue();

    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        REQUIRE(disparity != nullptr);
        auto depth = depthQueue->get<dai::ImgFrame>();
        REQUIRE(depth != nullptr);
    }
}

TEST_CASE("Test StereoDepth node ACCURACY preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::ACCURACY);
}

TEST_CASE("Test StereoDepth node DENSITY preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::DENSITY);
}

TEST_CASE("Test StereoDepth node DEFAULT preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::DEFAULT);
}

TEST_CASE("Test StereoDepth node FACE preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::FACE);
}

TEST_CASE("Test StereoDepth node HIGH_DETAIL preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::HIGH_DETAIL);
}

TEST_CASE("Test StereoDepth node ROBOTICS preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::ROBOTICS);
}

TEST_CASE("Test StereoDepth node CPU backend") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::DEFAULT, dai::ProcessorType::CPU);
}

TEST_CASE("Test StereoDepth node DSP backend") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::DEFAULT, dai::ProcessorType::DSP);
}
