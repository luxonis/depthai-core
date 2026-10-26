#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

namespace {

void checkInterpolatedPixels(dai::Pipeline& pipeline, bool runOnHost) {
    struct Queues {
        dai::Interpolation interpolation;
        std::shared_ptr<dai::InputQueue> input;
        std::shared_ptr<dai::InputQueue> reference;
        std::shared_ptr<dai::MessageQueue> output;
    };
    std::vector<Queues> queues;
    for(auto interpolation : {dai::Interpolation::AUTO, dai::Interpolation::NEAREST_NEIGHBOR, dai::Interpolation::BILINEAR, dai::Interpolation::BICUBIC}) {
        auto align = pipeline.create<dai::node::Align>();
        align->setRunOnHost(runOnHost);
        // Isolate image sampling from the independent per-pixel depth reprojection.
        align->initialConfig->staticDepthPlane = 1000;
        align->setInterpolation(interpolation);
        queues.push_back({interpolation, align->input.createInputQueue(), align->inputAlignTo.createInputQueue(), align->outputAligned.createOutputQueue()});
    }
    const std::array<std::array<float, 3>, 3> intrinsics = {{{8, 0, 4}, {0, 8, 4}, {0, 0, 1}}};
    const dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    dai::ImgTransformation transform(8, 8, intrinsics, dai::CameraModel::Perspective, {}, extrinsics);
    auto target = transform;
    target.addScale(2, 2);
    auto reference = std::make_shared<dai::ImgFrame>();
    reference->setWidth(16).setHeight(16).setType(dai::ImgFrame::Type::GRAY8);
    reference->setData(std::vector<uint8_t>(256));
    reference->setTransformation(target);
    const std::array<uint8_t, 8> row = {30, 60, 180, 220, 80, 100, 190, 30};
    std::vector<uint8_t> pixels;
    for(int y = 0; y < 8; ++y) pixels.insert(pixels.end(), row.begin(), row.end());
    pipeline.start();
    for(const auto& q : queues) {
        CAPTURE(runOnHost, q.interpolation);
        auto image = std::make_shared<dai::ImgFrame>();
        image->setWidth(8).setHeight(8).setType(dai::ImgFrame::Type::GRAY8);
        image->setData(pixels);
        image->setTransformation(transform);
        q.reference->send(reference);
        q.input->send(image);
        bool timedOut = false;
        auto output = q.output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output != nullptr);
        // At x=3 the source coordinate is 1.5. Cubic uses four samples,
        // linear uses two, and nearest selects source x=2.
        const int expected = q.interpolation == dai::Interpolation::BICUBIC ? 119 : q.interpolation == dai::Interpolation::BILINEAR ? 120 : 180;
        REQUIRE(output->getData()[8 * output->getStride() + 3] == expected);
        auto mask = std::make_shared<dai::SegmentationMask>();
        mask->setMask(pixels, 8, 8);
        mask->setTransformation(transform);
        q.reference->send(reference);
        q.input->send(mask);
        auto alignedMask = q.output->get<dai::SegmentationMask>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(alignedMask != nullptr);
        const auto maskFrame = alignedMask->getFrame();
        REQUIRE(maskFrame.getData()[8 * maskFrame.getStride() + 3] == 180);
        std::vector<uint8_t> depthPixels;
        for(auto value : pixels) {
            depthPixels.push_back(value);
            depthPixels.push_back(0);
        }
        auto depth = std::make_shared<dai::ImgFrame>();
        depth->setWidth(8).setHeight(8).setType(dai::ImgFrame::Type::RAW16);
        depth->setData(depthPixels);
        depth->setTransformation(transform);
        q.reference->send(reference);
        q.input->send(depth);
        auto alignedDepth = q.output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(alignedDepth != nullptr);
        REQUIRE(alignedDepth->getData()[8 * alignedDepth->getStride() + 6] == 180);
        REQUIRE(alignedDepth->getData()[8 * alignedDepth->getStride() + 7] == 0);
    }
    pipeline.stop();
    pipeline.wait();
}

}  // namespace

TEST_CASE("Align interpolation properties roundtrip", "[interpolation][serialization]") {
    for(auto interpolation : {dai::Interpolation::AUTO, dai::Interpolation::NEAREST_NEIGHBOR, dai::Interpolation::BILINEAR, dai::Interpolation::BICUBIC}) {
        dai::Pipeline pipeline(false);
        auto align = pipeline.create<dai::node::Align>();
        REQUIRE(&align->setInterpolation(interpolation) == align.get());
        auto bytes = dai::utility::serialize(align->properties);
        dai::AlignProperties restored;
        REQUIRE(dai::utility::deserialize(bytes, restored));
        REQUIRE(restored.interpolation == interpolation);
    }
    dai::Pipeline pipeline(false);
    auto align = pipeline.create<dai::node::Align>();
    REQUIRE_THROWS_AS(align->setInterpolation(static_cast<dai::Interpolation>(999)), std::invalid_argument);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
TEST_CASE("Align applies selected interpolation to pixels and preserves depth and labels on host", "[interpolation][pixels]") {
    dai::Pipeline pipeline(false);
    checkInterpolatedPixels(pipeline, true);
}
#endif

TEST_CASE("Align applies selected interpolation to pixels and preserves depth and labels on device", "[interpolation][pixels]") {
    dai::Pipeline pipeline;
    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) SKIP("Device Align requires RVC4");
    checkInterpolatedPixels(pipeline, false);
}
