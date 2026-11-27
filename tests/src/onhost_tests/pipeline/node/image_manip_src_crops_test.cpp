#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
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
constexpr auto OUTPUT_TIMEOUT = std::chrono::seconds(10);

std::shared_ptr<dai::ImgFrame> makeScaledFrame() {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(INPUT_WIDTH);
    frame->setHeight(INPUT_HEIGHT);
    frame->setType(dai::ImgFrame::Type::GRAY8);
    frame->setData(std::vector<uint8_t>(INPUT_WIDTH * INPUT_HEIGHT, 128));
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
    return frame;
}

}  // namespace

TEST_CASE("Host ImageManip letterbox of a scaled frame masks only the padding", "[ImageManip][ImgTransformation]") {
    // A 4:3 input in a 2:1 output: the image is 160 pixels wide, with 20 pixel bars left and right
    constexpr unsigned WIDTH = 200, HEIGHT = 120;
    auto frame = runImageManip(WIDTH, HEIGHT, dai::ImageManipConfig::ResizeMode::LETTERBOX);
    auto transformation = frame->transformation;
    for(unsigned y : {3U, 60U, 116U}) {
        INFO("y=" << y);
        CHECK_FALSE(transformation.getDstMaskPt(5, y));
        CHECK_FALSE(transformation.getDstMaskPt(194, y));
        CHECK(transformation.getDstMaskPt(26, y));
        CHECK(transformation.getDstMaskPt(100, y));
        CHECK(transformation.getDstMaskPt(173, y));
    }
}
