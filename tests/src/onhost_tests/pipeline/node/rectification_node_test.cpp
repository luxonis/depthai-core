#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

std::shared_ptr<dai::ImgFrame> makeFrame(
    unsigned int width, unsigned int height, dai::ImgFrame::Type type, std::vector<std::uint8_t> data, std::int64_t sequenceNum) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setData(std::move(data));
    frame->setType(type);
    frame->setSize(width, height);
    frame->setStride(static_cast<unsigned int>(width * frame->getBytesPerPixel()) + 3);
    frame->setSourceSize(width + 4, height + 2);
    frame->setTransformation(dai::ImgTransformation(width + 4, height + 2, width, height));
    frame->setInstanceNum(static_cast<unsigned int>(sequenceNum + 10));
    frame->setCategory(static_cast<unsigned int>(sequenceNum + 20));
    frame->setSequenceNum(sequenceNum);
    frame->setTimestamp(std::chrono::steady_clock::time_point(std::chrono::milliseconds(sequenceNum * 10)));
    frame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(sequenceNum * 20)));
    frame->setTimestampSystem(std::chrono::system_clock::time_point(std::chrono::milliseconds(sequenceNum * 30)));
    return frame;
}

void requireFrameUnchanged(const std::shared_ptr<dai::ImgFrame>& expected, const std::shared_ptr<dai::ImgFrame>& actual) {
    REQUIRE(actual != nullptr);
    REQUIRE(std::vector<std::uint8_t>(actual->getData().begin(), actual->getData().end())
            == std::vector<std::uint8_t>(expected->getData().begin(), expected->getData().end()));

    std::vector<std::uint8_t> expectedMetadata;
    std::vector<std::uint8_t> actualMetadata;
    dai::DatatypeEnum expectedDatatype;
    dai::DatatypeEnum actualDatatype;
    expected->serialize(expectedMetadata, expectedDatatype);
    actual->serialize(actualMetadata, actualDatatype);

    REQUIRE(actualDatatype == expectedDatatype);
    REQUIRE(actualMetadata == expectedMetadata);
}

std::shared_ptr<dai::ImgFrame> getFrame(const std::shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto frame = queue->get<dai::ImgFrame>(std::chrono::seconds(1), timedOut);
    REQUIRE_FALSE(timedOut);
    return frame;
}

}

TEST_CASE("Rectification disabled forwards images and metadata unchanged without extrinsics", "[Rectification]") {
    dai::Pipeline pipeline(false);
    auto rectification = pipeline.create<dai::node::Rectification>();
    rectification->setRunOnHost(true);
    rectification->enableRectification(false);
    rectification->setOutputSize(1, 1);

    auto input1Queue = rectification->input1.createInputQueue();
    auto input2Queue = rectification->input2.createInputQueue();
    auto output1Queue = rectification->output1.createOutputQueue();
    auto output2Queue = rectification->output2.createOutputQueue();

    pipeline.start();

    auto input1 = makeFrame(3, 2, dai::ImgFrame::Type::BGR888i, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}, 7);
    auto input2 = makeFrame(2, 3, dai::ImgFrame::Type::RAW16, {18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7}, 8);
    REQUIRE(input1->transformation.getExtrinsics().toCameraSocket == dai::CameraBoardSocket::AUTO);
    REQUIRE(input2->transformation.getExtrinsics().toCameraSocket == dai::CameraBoardSocket::AUTO);

    input1Queue->send(input1);
    input2Queue->send(input2);

    requireFrameUnchanged(input1, getFrame(output1Queue));
    requireFrameUnchanged(input2, getFrame(output2Queue));
}
