#include <algorithm>
#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"
#include "depthai/utility/CompilerWarnings.hpp"

void testStereoDepthPreset(dai::node::StereoDepth::PresetMode preset, dai::ProcessorType backend = dai::ProcessorType::CPU) {
    dai::Pipeline p;
    auto left = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = p.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = p.create<dai::node::StereoDepth>()->build(*left->requestOutput(std::make_pair(640, 400)), *right->requestOutput(std::make_pair(640, 400)));

    stereo->setDefaultProfilePreset(preset);
    stereo->initialConfig->setFiltersComputeBackend(backend);

    auto disparityQueue = stereo->disparity.createOutputQueue();
    auto depthQueue = stereo->depth.createOutputQueue();

    const bool rvc2ng = p.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG;
    if(rvc2ng && backend == dai::ProcessorType::DSP) {
        // RVC2NG has no DSP filter executor. The device must reject the request rather than fall back to the CPU.
        REQUIRE_THROWS_WITH(p.start(), Catch::Matchers::ContainsSubstring("DSP"));
        return;
    }

    p.start();

    // Retrieve N messages
    constexpr size_t N = 20;
    for(size_t i = 0; i < N; ++i) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        REQUIRE(disparity != nullptr);
        auto depth = depthQueue->get<dai::ImgFrame>();
        REQUIRE(depth != nullptr);
        if(rvc2ng) {
            // RAW8 carries the disparity only while its largest encoded value fits in eight bits.
            const auto& config = *stereo->initialConfig;
            const float maxDisparity = config.getMaxDisparity();
            const bool wide = maxDisparity > 255.0f;
            REQUIRE(disparity->getType() == (wide ? dai::ImgFrame::Type::RAW16 : dai::ImgFrame::Type::RAW8));
            REQUIRE(depth->getType() == dai::ImgFrame::Type::RAW16);
            REQUIRE(depth->getWidth() == disparity->getWidth());
            REQUIRE(depth->getHeight() == disparity->getHeight());
            const auto data = disparity->getData();
            const size_t bytesPerPixel = wide ? 2 : 1;
            REQUIRE(data.size() >= size_t(disparity->getWidth()) * disparity->getHeight() * bytesPerPixel);
            float largest = 0;
            for(size_t pixel = 0; pixel < size_t(disparity->getWidth()) * disparity->getHeight(); ++pixel) {
                const float value = wide ? float(data[pixel * 2] | (data[pixel * 2 + 1] << 8)) : float(data[pixel]);
                largest = std::max(largest, value);
            }
            REQUIRE(largest <= maxDisparity);
        }
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

TEST_CASE("StereoDepth running while repeatedly setting calibration does not crash device") {
    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    auto syncedLeftQueue = stereo->syncedLeft.createOutputQueue();
    auto disparityQueue = stereo->disparity.createOutputQueue();

    pipeline.start();

    constexpr int kNumFrames = 20;
    int receivedFrames = 0;

    for(int i = 0; i < kNumFrames; ++i) {
        if(!pipeline.isRunning()) {
        }

        std::shared_ptr<dai::ImgFrame> leftFrame;
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        try {
            leftFrame = syncedLeftQueue->get<dai::ImgFrame>();
        } catch(const std::exception& e) {
            FAIL(std::string("Failed to get syncedLeft frame: ") + e.what());
        }

        REQUIRE(leftFrame != nullptr);

        // This is the line that crashes in Python
        try {
            pipeline.getDefaultDevice()->setCalibration(pipeline.getDefaultDevice()->getCalibration());
        } catch(const std::exception& e) {
            FAIL(std::string("setCalibration() threw exception: ") + e.what());
        }

        receivedFrames++;
    }

    REQUIRE(receivedFrames == kNumFrames);
}