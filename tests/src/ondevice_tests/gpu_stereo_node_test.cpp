#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <random>

#include "depthai/depthai.hpp"

TEST_CASE("GPUStereo produces disparity on supported devices", "[GPUStereo][ondevice]") {
    std::shared_ptr<dai::Device> device;
    try {
        device = std::make_shared<dai::Device>();
    } catch(const std::exception& e) {
        WARN(std::string("Skipping GPUStereo test: no device available: ") + e.what());
        return;
    }

    if(!device->hasGPU()) {
        WARN("Skipping GPUStereo test: GPU not available on this device");
        return;
    }

    dai::Pipeline pipeline(device);

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 30.0f);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, 30.0f);

    auto leftOut = monoLeft->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);
    auto rightOut = monoRight->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);

    auto gpu = pipeline.create<dai::node::GPUStereo>();
    leftOut->link(gpu->left);
    rightOut->link(gpu->right);
    gpu->setRectification(true);
    gpu->initialConfig->setConfidenceThreshold(10);

    auto disparityQ = gpu->disparity.createOutputQueue();

    pipeline.start();

    constexpr int kNumFrames = 10;
    for(int i = 0; i < kNumFrames; ++i) {
        auto frame = disparityQ->get<dai::ImgFrame>();
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getWidth() > 0);
        REQUIRE(frame->getHeight() > 0);
        REQUIRE_FALSE(frame->getData().empty());
    }
}

TEST_CASE("StereoDepth GPU backends preserve the GPUStereo algorithm", "[StereoDepth][GPUStereo][ondevice]") {
    const auto backend = GENERATE(dai::node::StereoDepth::Backend::AUTO, dai::node::StereoDepth::Backend::GPU, dai::node::StereoDepth::Backend::GPU_CPU);
    const auto inputType = GENERATE(dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::NV12, dai::ImgFrame::Type::YUV420p);
    auto device = std::make_shared<dai::Device>();
    if(!device->isGpuStereoSupported()) {
        SKIP("Device does not support GPU stereo");
    }
    dai::Pipeline pipeline(device);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->setBackend(backend);
    stereo->setRectification(false);
    stereo->setBaseline(7.5f);
    stereo->setFocalLength(400.0f);
    stereo->initialConfig->setDepthUnit(dai::DepthUnit::CENTIMETER);
    stereo->initialConfig->setConfidenceThreshold(10);
    stereo->initialConfig->postProcessing.temporalFilter.enable = false;

    auto legacy = pipeline.create<dai::node::GPUStereo>();
    legacy->setRectification(false);
    legacy->initialConfig->setConfidenceThreshold(10);
    legacy->initialConfig->setTemporalFilter(false);
    legacy->initialConfig->setComputeMode(backend == dai::node::StereoDepth::Backend::GPU ? dai::GPUStereoConfig::ComputeMode::GPU_ONLY
                                                                                          : dai::GPUStereoConfig::ComputeMode::GPU_CPU);
    auto left = stereo->left.createInputQueue();
    auto right = stereo->right.createInputQueue();
    auto legacyLeft = legacy->left.createInputQueue();
    auto legacyRight = legacy->right.createInputQueue();
    auto disparity = stereo->disparity.createOutputQueue();
    auto depth = stereo->depth.createOutputQueue();
    auto confidence = stereo->confidenceMap.createOutputQueue();
    auto synced = stereo->syncedLeft.createOutputQueue();
    auto rectified = stereo->rectifiedRight.createOutputQueue();
    auto reference = legacy->disparity.createOutputQueue();

    // Dense, deterministic texture with a known 12-pixel horizontal shift.
    constexpr int width = 640, height = 400;
    std::mt19937 rng(42);
    std::vector<uint8_t> leftData(width * height), rightData(width * height);
    for(auto& pixel : leftData) pixel = static_cast<uint8_t>(rng());
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) rightData[y * width + x] = leftData[y * width + std::min(x + 12, width - 1)];
    }
    pipeline.start();
    REQUIRE(stereo->getBackend() == backend);
    for(int sequence = 0; sequence < 4; ++sequence) {
        auto makeFrame = [&](const std::vector<uint8_t>& data) {
            auto frame = std::make_shared<dai::ImgFrame>();
            auto planes = data;
            if(inputType != dai::ImgFrame::Type::GRAY8) planes.resize(data.size() * 3 / 2, 128);
            frame->setData(std::move(planes));
            frame->setWidth(width);
            frame->setHeight(height);
            frame->setStride(width);
            frame->setType(inputType);
            frame->setSequenceNum(sequence);
            frame->setTimestamp(std::chrono::steady_clock::now());
            frame->setTimestampDevice(frame->getTimestamp());
            return frame;
        };
        auto l = makeFrame(leftData), r = makeFrame(rightData);
        r->setTimestamp(l->getTimestamp());
        r->setTimestampDevice(l->getTimestampDevice());
        if(sequence == 0) {
            auto stale = makeFrame(rightData);
            stale->setTimestampDevice(l->getTimestampDevice() - std::chrono::milliseconds(100));
            right->send(stale);
        }
        left->send(l);
        right->send(r);
        auto legacyL = std::make_shared<dai::ImgFrame>(*l), legacyR = std::make_shared<dai::ImgFrame>(*r);
        legacyL->setData(std::vector<uint8_t>(leftData));
        legacyL->setType(dai::ImgFrame::Type::GRAY8);
        legacyR->setData(std::vector<uint8_t>(rightData));
        legacyR->setType(dai::ImgFrame::Type::GRAY8);
        legacyLeft->send(legacyL);
        legacyRight->send(legacyR);
        auto receive = [](const std::shared_ptr<dai::MessageQueue>& queue) {
            bool timedOut = false;
            auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(30), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(frame != nullptr);
            return frame;
        };
        auto d = receive(disparity);
        auto expected = receive(reference);
        REQUIRE(std::equal(d->getData().begin(), d->getData().end(), expected->getData().begin(), expected->getData().end()));
        REQUIRE(d->getType() == dai::ImgFrame::Type::RAW16);
        REQUIRE(d->getSequenceNum() == sequence);
        REQUIRE(d->getWidth() == width);
        REQUIRE(d->getHeight() == height);
        auto syncedFrame = receive(synced), rectifiedFrame = receive(rectified);
        // Transport buffers can contain padded rows beyond the logical image.
        REQUIRE(syncedFrame->getData().size() >= leftData.size());
        REQUIRE(rectifiedFrame->getData().size() >= rightData.size());
        REQUIRE(std::equal(leftData.begin(), leftData.end(), syncedFrame->getData().begin()));
        REQUIRE(std::equal(rightData.begin(), rightData.end(), rectifiedFrame->getData().begin()));
        auto z = receive(depth);
        auto c = receive(confidence);
        const auto* disparities = reinterpret_cast<const uint16_t*>(d->getData().data());
        const auto* depths = reinterpret_cast<const uint16_t*>(z->getData().data());
        REQUIRE(d->getData().size() == width * height * sizeof(uint16_t));
        REQUIRE(z->getData().size() == width * height * sizeof(uint16_t));
        REQUIRE(c->getData().size() == width * height);
        int valid = 0;
        bool depthMatches = true, confidenceMatches = true;
        for(int i = 0; i < width * height; ++i) {
            if(disparities[i] == 0) {
                depthMatches &= depths[i] == 0;
            } else {
                ++valid;
                confidenceMatches &= c->getData()[i] >= 10;
                // 7.5 cm baseline * 400 px focal * 16 disparity units/px.
                depthMatches &= depths[i] == static_cast<uint16_t>(48000.0f / disparities[i]);
            }
        }
        REQUIRE(valid > width * height / 2);
        REQUIRE(depthMatches);
        REQUIRE(confidenceMatches);
    }
}

TEST_CASE("StereoDepth rejects matcher-side rectification on RVC4", "[StereoDepth][Rectification][ondevice]") {
    auto device = std::make_shared<dai::Device>();
    if(device->getPlatform() != dai::Platform::RVC4) {
        WARN("This rectification contract applies to RVC4");
        return;
    }
    struct InvalidMatcherRectification : dai::node::StereoDepth {
        Properties& getProperties() override {
            auto& props = dai::node::StereoDepth::getProperties();
            // Simulate an older SDK requesting rectification inside the matcher.
            props.enableRectification = true;
            return props;
        }
    };
    dai::Pipeline pipeline(device);
    auto stereo = std::make_shared<InvalidMatcherRectification>();
    pipeline.add(stereo);
    stereo->buildInternal();
    stereo->setBackend(dai::node::StereoDepth::Backend::HW);
    stereo->setRectification(false);
    REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("rectification must be handled by its Rectification subnode"));
}
