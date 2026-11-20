#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

// Host-only pipeline: a frame that is already scaled down from its sensor goes through ImageManip.
// The output transformation must still report which output pixels are covered by the image.

namespace {

constexpr unsigned SOURCE_WIDTH = 800, SOURCE_HEIGHT = 600;
constexpr unsigned INPUT_WIDTH = 400, INPUT_HEIGHT = 300;
constexpr uint8_t INPUT_VALUE = 128;
constexpr auto OUTPUT_TIMEOUT = std::chrono::seconds(10);

std::shared_ptr<dai::ImgFrame> makeScaledFrame() {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(INPUT_WIDTH);
    frame->setHeight(INPUT_HEIGHT);
    frame->setType(dai::ImgFrame::Type::GRAY8);
    frame->setData(std::vector<uint8_t>(INPUT_WIDTH * INPUT_HEIGHT, INPUT_VALUE));
    // The way a camera publishes a binned or ISP-scaled output: full sensor as the source, half-size frame
    dai::ImgTransformation transformation(SOURCE_WIDTH, SOURCE_HEIGHT);
    transformation.addScale(static_cast<float>(INPUT_WIDTH) / SOURCE_WIDTH, static_cast<float>(INPUT_HEIGHT) / SOURCE_HEIGHT);
    frame->setTransformation(transformation);
    return frame;
}

std::shared_ptr<dai::ImgFrame> runImageManip(unsigned outputWidth, unsigned outputHeight, dai::ImageManipConfig::ResizeMode mode) {
    dai::Pipeline pipeline(false);
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->setRunOnHost(true);
    manip->initialConfig->setOutputSize(outputWidth, outputHeight, mode);
    auto input = manip->inputImage.createInputQueue();
    auto output = manip->out.createOutputQueue();
    pipeline.start();
    input->send(makeScaledFrame());
    bool timedOut = false;
    auto frame = output->get<dai::ImgFrame>(OUTPUT_TIMEOUT, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame != nullptr);
    pipeline.stop();
    // The image was really resized, not only its metadata
    REQUIRE(frame->getWidth() == outputWidth);
    REQUIRE(frame->getHeight() == outputHeight);
    REQUIRE(frame->getData().size() >= static_cast<size_t>(outputWidth) * outputHeight);
    return frame;
}

uint8_t pixelAt(const dai::ImgFrame& frame, unsigned x, unsigned y) {
    return frame.getData()[y * frame.getStride() + x];
}

}  // namespace

TEST_CASE("Host ImageManip output of a scaled frame is covered by its source", "[ImageManip][ImgTransformation]") {
    const auto mode = GENERATE(dai::ImageManipConfig::ResizeMode::STRETCH, dai::ImageManipConfig::ResizeMode::CENTER_CROP);
    constexpr unsigned WIDTH = 200, HEIGHT = 100;
    auto frame = runImageManip(WIDTH, HEIGHT, mode);
    auto transformation = frame->transformation;
    REQUIRE(transformation.getSize() == std::pair<size_t, size_t>(WIDTH, HEIGHT));
    CHECK(pixelAt(*frame, WIDTH / 2, HEIGHT / 2) == INPUT_VALUE);

    // Every output pixel shows a part of the sensor
    for(unsigned y = 2; y < HEIGHT; y += 12) {
        for(unsigned x = 2; x < WIDTH; x += 12) {
            INFO("x=" << x << " y=" << y);
            CHECK(transformation.getDstMaskPt(x, y));
        }
    }
    CHECK(transformation.getDstMaskPt(WIDTH - 3, HEIGHT - 3));
}

TEST_CASE("Host ImageManip letterbox of a scaled frame masks only the padding", "[ImageManip][ImgTransformation]") {
    // A 4:3 input in a 2:1 output: the image is 160 pixels wide, with 20 pixel bars left and right
    constexpr unsigned WIDTH = 200, HEIGHT = 120;
    auto frame = runImageManip(WIDTH, HEIGHT, dai::ImageManipConfig::ResizeMode::LETTERBOX);
    auto transformation = frame->transformation;
    REQUIRE(transformation.getSize() == std::pair<size_t, size_t>(WIDTH, HEIGHT));
    // The pixels agree with the mask: image in the middle, black bars on the sides
    CHECK(pixelAt(*frame, 100, 60) == INPUT_VALUE);
    CHECK(pixelAt(*frame, 5, 60) == 0);
    CHECK(pixelAt(*frame, 194, 60) == 0);

    for(unsigned y : {3U, 60U, 116U}) {
        INFO("y=" << y);
        CHECK_FALSE(transformation.getDstMaskPt(5, y));
        CHECK_FALSE(transformation.getDstMaskPt(194, y));
        CHECK(transformation.getDstMaskPt(26, y));
        CHECK(transformation.getDstMaskPt(100, y));
        CHECK(transformation.getDstMaskPt(173, y));
    }
}

TEST_CASE("A crop that covers only padding leaves no valid pixel", "[ImgTransformation]") {
    // A 200 pixel wide sensor scaled to 100 pixels and padded by 40 pixels on the right.
    // A crop of the padding at x = 110..130 of that frame is x = 220..260 on the sensor, outside of it.
    constexpr unsigned WIDTH = 200, HEIGHT = 100;
    dai::ImgTransformation transformation(WIDTH, HEIGHT);
    transformation.addScale(0.5f, 0.5f);
    transformation.addPadding(0, 0, 0, 40);
    const auto [width, height] = transformation.getSize();
    REQUIRE(width == 140);
    REQUIRE(height == 50);
    transformation.addSrcCrops({dai::RotatedRect(dai::Point2f(240.0f, 50.0f), dai::Size2f(40.0f, 100.0f), 0.0f)});

    for(unsigned y = 0; y <= HEIGHT; y += 10) {
        for(unsigned x = 0; x <= WIDTH + 80; x += 10) {
            INFO("x=" << x << " y=" << y);
            CHECK_FALSE(transformation.getSrcMaskPt(x, y));
            CHECK_FALSE(transformation.getDstMaskPt(x, y));
        }
    }
}
