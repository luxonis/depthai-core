#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

void testStereoDepthPreset(dai::node::StereoDepth::PresetMode preset) {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = p.create<dai::node::StereoDepth>()->build(*left->requestOutput(std::make_pair(640, 400)), *right->requestOutput(std::make_pair(640, 400)));

    stereo->setDefaultProfilePreset(preset);

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

TEST_CASE("Test StereoDepth node HIGH_ACCURACY preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
}

TEST_CASE("Test StereoDepth node HIGH_DENSITY preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
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

TEST_CASE("Test StereoDepth node HIGH_FPS preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::HIGH_FPS);
}

TEST_CASE("Test StereoDepth node HIGH_ACCURACY2 preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY2);
}

TEST_CASE("Test StereoDepth node ROBOTICS preset") {
    testStereoDepthPreset(dai::node::StereoDepth::PresetMode::ROBOTICS);
}
