#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/AprilTag.hpp"

namespace {

std::shared_ptr<dai::ImgFrame> makeFrame(dai::ImgFrame::Type type, bool padded) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setSize(24, 16).setType(type).setStride(padded ? 31 : 24);
    frame->fb.p1Offset = padded ? 13 : 0;
    frame->setData(std::vector<uint8_t>(frame->fb.p1Offset + frame->getStride() * frame->getHeight(), 255));
    frame->setSequenceNum(42);
    return frame;
}

void checkFrame(const std::shared_ptr<dai::ImgFrame>& frame, bool valid) {
    dai::Pipeline pipeline(false);
    auto detector = pipeline.create<dai::node::AprilTag>();
    auto input = detector->inputImage.createInputQueue();
    auto output = detector->out.createOutputQueue();
    pipeline.start();
    input->send(frame);
    bool timedOut = false;
    if(valid) {
        auto result = output->get<dai::AprilTags>(std::chrono::seconds(3), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(result != nullptr);
        REQUIRE(result->getSequenceNum() == 42);
        REQUIRE(result->aprilTags.empty());
    } else {
        REQUIRE_THROWS_AS(output->get<dai::AprilTags>(std::chrono::seconds(3), timedOut), dai::MessageQueue::QueueException);
    }
    pipeline.stop();
    pipeline.wait();
}

}  // namespace

TEST_CASE("AprilTag accepts complete packed and padded first planes", "[AprilTag][onhost]") {
    const auto type = GENERATE(dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::RAW8, dai::ImgFrame::Type::NV12);
    const bool padded = GENERATE(false, true);
    CAPTURE(type, padded);
    auto frame = makeFrame(type, padded);
    const auto data = frame->getData();
    const std::vector<uint8_t> original(data.begin(), data.end());
    // NV12 needs only its Y plane here; no chroma plane is consumed by the detector.
    checkFrame(frame, true);
    REQUIRE(std::equal(original.begin(), original.end(), frame->getData().begin()));
}

TEST_CASE("AprilTag rejects malformed first-plane storage and geometry", "[AprilTag][onhost]") {
    const auto type = GENERATE(dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::RAW8, dai::ImgFrame::Type::NV12);
    CAPTURE(type);
    auto frame = makeFrame(type, true);
    SECTION("metadata-only frame") {
        frame->setData(std::vector<uint8_t>{});
    }
    SECTION("missing backing storage") {
        frame->data.reset();
    }
    SECTION("offset beyond backing storage") {
        frame->fb.p1Offset = frame->getData().size() + 1;
    }
    SECTION("offset at the end of backing storage") {
        frame->fb.p1Offset = frame->getData().size();
    }
    SECTION("truncated visible pixels") {
        frame->data->setSize(frame->getData().size() - 8);
    }
    SECTION("truncated last-row padding") {
        frame->data->setSize(frame->getData().size() - 1);
    }
    SECTION("stride smaller than width") {
        frame->setStride(frame->getWidth() - 1);
    }
    SECTION("zero width") {
        frame->fb.width = 0;
    }
    SECTION("zero height") {
        frame->fb.height = 0;
    }
    SECTION("width outside detector integer range") {
        frame->fb.width = std::numeric_limits<uint32_t>::max();
    }
    SECTION("height outside detector integer range") {
        frame->fb.height = std::numeric_limits<uint32_t>::max();
    }
    SECTION("stride outside detector integer range") {
        frame->fb.stride = std::numeric_limits<uint32_t>::max();
    }
    SECTION("first-plane size overflows detector integer arithmetic") {
        frame->fb.stride = std::numeric_limits<int32_t>::max();
    }
    checkFrame(frame, false);
}
