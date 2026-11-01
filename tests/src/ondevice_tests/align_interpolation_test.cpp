#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

namespace {

// Input and reference geometry, the output pixel that falls between source
// pixels, and its expected value per interpolation.
struct Geometry {
    dai::ImgTransformation input;
    dai::ImgTransformation reference;
    int x;
    int y;
    int bicubic;
    int bilinear;
    int nearest;
};

std::vector<Geometry> geometries() {
    const std::array<std::array<float, 3>, 3> intrinsics = {{{8, 0, 4}, {0, 8, 4}, {0, 0, 1}}};
    const dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::MILLIMETER);
    const dai::ImgTransformation plain(8, 8, intrinsics, dai::CameraModel::Perspective, {}, extrinsics);
    auto upscaled = plain;
    upscaled.addScale(2, 2);
    std::vector<float> distortion(14, 0.0f);
    distortion[0] = -2;
    const dai::ImgTransformation distorted(8, 8, intrinsics, dai::CameraModel::Perspective, distortion, extrinsics);
    return {
        // Second remap: the 2x upscale samples source x=1.5 for output x=3. Cubic
        // uses four samples, linear uses two, and nearest selects source x=2.
        // The first remap is an identity.
        {plain, upscaled, 3, 8, 119, 120, 180},
        // First remap: undistorting with k1 = -2 samples output x=2 (normalized
        // -0.25, r^2 = 1/16) at source x = 4 + 8 * -0.25 * (1 - 2 / 16) = 2.25.
        // The second remap is an identity.
        {distorted, plain, 2, 4, 207, 190, 180},
    };
}

void checkInterpolatedPixels(dai::Pipeline& pipeline, bool runOnHost) {
    struct Queues {
        size_t geometry;
        dai::Interpolation interpolation;
        std::shared_ptr<dai::InputQueue> input;
        std::shared_ptr<dai::InputQueue> reference;
        std::shared_ptr<dai::MessageQueue> output;
    };
    const auto cases = geometries();
    std::vector<Queues> queues;
    for(size_t geometry = 0; geometry < cases.size(); ++geometry) {
        for(auto interpolation : {dai::Interpolation::AUTO, dai::Interpolation::NEAREST_NEIGHBOR, dai::Interpolation::BILINEAR, dai::Interpolation::BICUBIC}) {
            auto align = pipeline.create<dai::node::Align>();
            align->setRunOnHost(runOnHost);
            // Isolate image sampling from the independent per-pixel depth reprojection.
            align->initialConfig->staticDepthPlane = 1000;
            align->setInterpolation(interpolation);
            queues.push_back(
                {geometry, interpolation, align->input.createInputQueue(), align->inputAlignTo.createInputQueue(), align->outputAligned.createOutputQueue()});
        }
    }
    const std::array<uint8_t, 8> row = {30, 60, 180, 220, 80, 100, 190, 30};
    std::vector<uint8_t> pixels;
    for(int y = 0; y < 8; ++y) pixels.insert(pixels.end(), row.begin(), row.end());
    pipeline.start();
    for(const auto& q : queues) {
        CAPTURE(runOnHost, q.geometry, q.interpolation);
        const auto& geometry = cases[q.geometry];
        const auto [width, height] = geometry.reference.getSize();
        auto reference = std::make_shared<dai::ImgFrame>();
        reference->setWidth(width).setHeight(height).setType(dai::ImgFrame::Type::GRAY8);
        reference->setData(std::vector<uint8_t>(width * height));
        reference->setTransformation(geometry.reference);
        auto image = std::make_shared<dai::ImgFrame>();
        image->setWidth(8).setHeight(8).setType(dai::ImgFrame::Type::GRAY8);
        image->setData(pixels);
        image->setTransformation(geometry.input);
        q.reference->send(reference);
        q.input->send(image);
        bool timedOut = false;
        auto output = q.output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(output != nullptr);
        const int expected = q.interpolation == dai::Interpolation::BICUBIC    ? geometry.bicubic
                             : q.interpolation == dai::Interpolation::BILINEAR ? geometry.bilinear
                                                                               : geometry.nearest;
        REQUIRE(output->getData()[geometry.y * output->getStride() + geometry.x] == expected);
        auto mask = std::make_shared<dai::SegmentationMask>();
        mask->setMask(pixels, 8, 8);
        mask->setTransformation(geometry.input);
        q.reference->send(reference);
        q.input->send(mask);
        auto alignedMask = q.output->get<dai::SegmentationMask>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(alignedMask != nullptr);
        const auto maskFrame = alignedMask->getFrame();
        REQUIRE(maskFrame.getData()[geometry.y * maskFrame.getStride() + geometry.x] == geometry.nearest);
        std::vector<uint8_t> depthPixels;
        for(auto value : pixels) {
            depthPixels.push_back(value);
            depthPixels.push_back(0);
        }
        auto depth = std::make_shared<dai::ImgFrame>();
        depth->setWidth(8).setHeight(8).setType(dai::ImgFrame::Type::RAW16);
        depth->setData(depthPixels);
        depth->setTransformation(geometry.input);
        q.reference->send(reference);
        q.input->send(depth);
        auto alignedDepth = q.output->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(alignedDepth != nullptr);
        REQUIRE(alignedDepth->getData()[geometry.y * alignedDepth->getStride() + 2 * geometry.x] == geometry.nearest);
        REQUIRE(alignedDepth->getData()[geometry.y * alignedDepth->getStride() + 2 * geometry.x + 1] == 0);
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
