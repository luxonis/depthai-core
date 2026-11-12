/**
 * StereoDepth presets on frames sent from the host, so no stereo camera pair is needed.
 *
 * The left frame is random texture and the right frame is the same texture shifted by 12 px.
 * With a 450 px focal length and a 7.5 cm baseline, every valid pixel should read about 2812 mm.
 * Calibration is pipeline-local; the device EEPROM is never written.
 */
#include <catch2/catch_all.hpp>
#include <chrono>
#include <random>

#include "depthai/depthai.hpp"

namespace {
using Preset = dai::node::StereoDepth::PresetMode;
constexpr unsigned WIDTH = 640, HEIGHT = 400, SHIFT = 12;
constexpr float FOCAL = 450, BASELINE_MM = 75, EXPECTED_MM = FOCAL * BASELINE_MM / SHIFT;

dai::CalibrationHandler calibration() {
    dai::CalibrationHandler handler;
    const std::vector<std::vector<float>> intrinsics{{FOCAL, 0, WIDTH / 2.0f}, {0, FOCAL, HEIGHT / 2.0f}, {0, 0, 1}};
    const std::vector<std::vector<float>> identity{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
        handler.setCameraIntrinsics(socket, intrinsics, WIDTH, HEIGHT);
        handler.setDistortionCoefficients(socket, std::vector<float>(14, 0));
        handler.setFov(socket, 70);
    }
    const std::vector<float> translation{-BASELINE_MM / 10, 0, 0};
    handler.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, identity, translation, translation);
    handler.setStereoLeft(dai::CameraBoardSocket::CAM_B, identity);
    handler.setStereoRight(dai::CameraBoardSocket::CAM_C, identity);
    return handler;
}

struct Scene {
    std::vector<uint8_t> left, right;
    explicit Scene(dai::Pipeline& pipeline) {
        // RVC4 stereo reads complete 32-row blocks, so there the storage is padded to whole blocks while the frames
        // stay HEIGHT rows tall.
        const bool rvc4 = pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC4;
        const unsigned rows = rvc4 ? (HEIGHT + 31) / 32 * 32 : HEIGHT;
        left.assign(WIDTH * rows, 0);
        right.assign(WIDTH * rows, 0);
        std::mt19937 random(7);
        for(unsigned i = 0; i < WIDTH * HEIGHT; ++i) left[i] = random() & 255;
        for(unsigned y = 0; y < HEIGHT; ++y)
            for(unsigned x = 0; x + SHIFT < WIDTH; ++x) right[y * WIDTH + x] = left[y * WIDTH + x + SHIFT];
    }
};

std::shared_ptr<dai::ImgFrame> frame(dai::CameraBoardSocket socket, const std::vector<uint8_t>& pixels, int sequence) {
    auto image = std::make_shared<dai::ImgFrame>();
    image->setType(dai::ImgFrame::Type::GRAY8).setSize(WIDTH, HEIGHT).setStride(WIDTH).setInstanceNum(unsigned(socket)).setSequenceNum(sequence);
    image->setData(pixels);
    return image;
}

struct Outputs {
    std::shared_ptr<dai::ImgFrame> disparity, depth;
};

Outputs process(dai::InputQueue& left, dai::InputQueue& right, dai::MessageQueue& disparities, dai::MessageQueue& depths, const Scene& scene, int sequence) {
    left.send(frame(dai::CameraBoardSocket::CAM_B, scene.left, sequence));
    right.send(frame(dai::CameraBoardSocket::CAM_C, scene.right, sequence));
    bool disparityTimedOut = false, depthTimedOut = false;
    Outputs outputs{disparities.get<dai::ImgFrame>(std::chrono::seconds(10), disparityTimedOut),
                    depths.get<dai::ImgFrame>(std::chrono::seconds(10), depthTimedOut)};
    REQUIRE_FALSE(disparityTimedOut);
    REQUIRE_FALSE(depthTimedOut);
    REQUIRE(outputs.disparity != nullptr);
    REQUIRE(outputs.depth != nullptr);
    return outputs;
}

/** Returns the share of valid depth pixels within 4 % of the expected distance, and the share of valid pixels. */
std::pair<double, double> depthAccuracy(const dai::ImgFrame& depth) {
    const auto data = depth.getData();
    const size_t pixels = size_t(depth.getWidth()) * depth.getHeight();
    REQUIRE(data.size() >= pixels * 2);
    size_t valid = 0, correct = 0;
    for(size_t i = 0; i < pixels; ++i) {
        const unsigned z = data[2 * i] | (data[2 * i + 1] << 8);
        if(!z) continue;
        ++valid;
        if(std::abs(float(z) - EXPECTED_MM) < EXPECTED_MM * 0.04f) ++correct;
    }
    return {valid ? double(correct) / valid : 0, double(valid) / pixels};
}

bool rvc2ng(dai::Pipeline& pipeline) {
    return pipeline.getDefaultDevice()->getPlatform() == dai::Platform::RVC2NG;
}
}  // namespace

TEST_CASE("StereoDepth presets produce correct depth from host frames", "[stereo-host-frames]") {
    const auto [preset, name] = GENERATE(table<Preset, std::string>({{Preset::DEFAULT, "DEFAULT"},
                                                                    {Preset::FACE, "FACE"},
                                                                    {Preset::HIGH_DETAIL, "HIGH_DETAIL"},
                                                                    {Preset::ROBOTICS, "ROBOTICS"},
                                                                    {Preset::DENSITY, "DENSITY"},
                                                                    {Preset::ACCURACY, "ACCURACY"},
                                                                    {Preset::FAST_DENSITY, "FAST_DENSITY"},
                                                                    {Preset::FAST_ACCURACY, "FAST_ACCURACY"}}));
    CAPTURE(name);
    dai::Pipeline pipeline;
    pipeline.setCalibrationData(calibration());
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(preset);
    if(rvc2ng(pipeline)) {
        // RVC2NG runs these filters on the CPU, so its presets leave them off.
        const auto& filters = stereo->initialConfig->postProcessing;
        REQUIRE(filters.median == dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF);
        REQUIRE_FALSE(filters.spatialFilter.enable);
        REQUIRE_FALSE(filters.temporalFilter.enable);
    }
    auto left = stereo->left.createInputQueue(), right = stereo->right.createInputQueue();
    auto disparities = stereo->disparity.createOutputQueue(), depths = stereo->depth.createOutputQueue();
    pipeline.start();
    const Scene scene(pipeline);
    for(int sequence = 1; sequence <= 3; ++sequence) {
        CAPTURE(sequence);
        const auto outputs = process(*left, *right, *disparities, *depths, scene, sequence);
        const float maxDisparity = stereo->initialConfig->getMaxDisparity();
        // RAW8 carries the disparity only while its largest encoded value fits in eight bits.
        REQUIRE(outputs.disparity->getType() == (maxDisparity > 255 ? dai::ImgFrame::Type::RAW16 : dai::ImgFrame::Type::RAW8));
        REQUIRE(outputs.depth->getType() == dai::ImgFrame::Type::RAW16);
        REQUIRE(outputs.depth->getWidth() == outputs.disparity->getWidth());
        REQUIRE(outputs.depth->getHeight() == outputs.disparity->getHeight());
        const auto [correct, valid] = depthAccuracy(*outputs.depth);
        CAPTURE(correct, valid);
        REQUIRE(valid > 0.5);
        REQUIRE(correct > 0.9);
    }
    pipeline.stop();
}

TEST_CASE("StereoDepth rejects the DSP filter backend without a DSP executor", "[stereo-host-frames]") {
    dai::Pipeline pipeline;
    if(!rvc2ng(pipeline)) SKIP("Only RVC2NG lacks a DSP filter executor");
    pipeline.setCalibrationData(calibration());
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(Preset::DEFAULT);
    stereo->initialConfig->setFiltersComputeBackend(dai::ProcessorType::DSP);
    auto left = stereo->left.createInputQueue(), right = stereo->right.createInputQueue();
    auto disparities = stereo->disparity.createOutputQueue();
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("DSP"));
}

TEST_CASE("StereoDepth keeps running after rejecting a runtime config", "[stereo-host-frames]") {
    dai::Pipeline pipeline;
    if(!rvc2ng(pipeline)) SKIP("The rejected setting is a confidence threshold, which only RVC2NG cannot honor");
    pipeline.setCalibrationData(calibration());
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setDefaultProfilePreset(Preset::DEFAULT);
    auto left = stereo->left.createInputQueue(), right = stereo->right.createInputQueue();
    auto configs = stereo->inputConfig.createInputQueue();
    auto disparities = stereo->disparity.createOutputQueue(), depths = stereo->depth.createOutputQueue();
    auto applied = stereo->outConfig.createOutputQueue();
    pipeline.start();
    const Scene scene(pipeline);
    process(*left, *right, *disparities, *depths, scene, 1);
    REQUIRE(applied->get<dai::StereoDepthConfig>() != nullptr);

    // Each config carries a distinct depth limit, so outConfig shows which one a frame used.
    const auto previousRange = stereo->initialConfig->postProcessing.thresholdFilter.maxRange;
    const auto previousThreshold = stereo->initialConfig->costMatching.confidenceThreshold;
    REQUIRE(previousRange > EXPECTED_MM);
    auto rejected = std::make_shared<dai::StereoDepthConfig>(*stereo->initialConfig);
    rejected->setConfidenceThreshold(200);
    rejected->postProcessing.thresholdFilter.maxRange = 1000;
    configs->send(rejected);
    // Configs and frames travel on separate queues, and the device reads at most one config per frame. Two frames give
    // the rejected config time to arrive and be read; the frames after it still use the previous config.
    for(int sequence = 2; sequence <= 3; ++sequence) {
        CAPTURE(sequence);
        const auto outputs = process(*left, *right, *disparities, *depths, scene, sequence);
        const auto used = applied->get<dai::StereoDepthConfig>();
        REQUIRE(used != nullptr);
        REQUIRE(used->postProcessing.thresholdFilter.maxRange == previousRange);
        REQUIRE(used->costMatching.confidenceThreshold == previousThreshold);
        REQUIRE(depthAccuracy(*outputs.depth).second > 0.5);
    }

    // The device consumes configs in order: once the valid 2 m limit is in use, the rejected 1 m one was already seen.
    auto accepted = std::make_shared<dai::StereoDepthConfig>(*stereo->initialConfig);
    accepted->postProcessing.thresholdFilter.maxRange = 2000;
    configs->send(accepted);
    bool appliedValid = false;
    for(int sequence = 4; sequence <= 40 && !appliedValid; ++sequence) {
        CAPTURE(sequence);
        const auto outputs = process(*left, *right, *disparities, *depths, scene, sequence);
        const auto used = applied->get<dai::StereoDepthConfig>();
        REQUIRE(used != nullptr);
        const auto maxRange = used->postProcessing.thresholdFilter.maxRange;
        CAPTURE(maxRange, previousRange);
        REQUIRE((maxRange == previousRange || maxRange == 2000));
        REQUIRE(used->costMatching.confidenceThreshold == previousThreshold);
        // The scene is 2.8 m away, so the 2 m limit removes all of it and the previous config keeps most of it.
        const double valid = depthAccuracy(*outputs.depth).second;
        if(maxRange == 2000)
            REQUIRE(valid == 0);
        else
            REQUIRE(valid > 0.5);
        appliedValid = maxRange == 2000;
    }
    REQUIRE(appliedValid);
    REQUIRE(pipeline.isRunning());
    pipeline.stop();
}
