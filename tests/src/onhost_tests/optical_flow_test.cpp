#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstring>
#include <limits>

#include "../../../device/rvc4/src/utilities/OpticalFlowInput.hpp"
#include "../../../device/rvc4/src/utilities/OpticalFlowOutput.hpp"
#include "depthai/pipeline/node/OpticalFlow.hpp"
#include "depthai/utility/OffsetMemory.hpp"

TEST_CASE("Native optical flow preserves signed fixed point, pixel order and buffer ownership", "[OpticalFlow]") {
    constexpr unsigned int width = 128, height = 32;
    const std::size_t pixels = width * height;
    std::vector<int16_t> forward(2 * pixels, 0), backward(2 * pixels, 0);
    std::vector<uint8_t> confidence(pixels, 255);
    forward[0] = 24;
    forward[1] = 0x0fe8;                         // -1.5, zero-padded signed 12-bit
    backward[0] = static_cast<int16_t>(0xffe8);  // same value, sign extended
    forward[2 * (width + 3)] = 64;
    confidence[width + 3] = 0;  // low confidence is preserved, not masked
    dai::ImgFrame reference, target;
    reference.setWidth(width).setHeight(height).setSequenceNum(100);
    target.setWidth(width).setHeight(height).setSequenceNum(103);
    reference.setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(1234567890)));
    target.setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::nanoseconds(1334567891)));
    auto output = dai::gate::makeOpticalFlowOutput(forward.data(), backward.data(), confidence.data(), reference, target);
    std::fill(forward.begin(), forward.end(), 0);
    std::fill(confidence.begin(), confidence.end(), 0);
    auto flow = output->get<dai::NNData>("flow");
    auto fwd = flow->getTensorInfo("forward").value();
    auto bwd = flow->getTensorInfo("backward").value();
    auto conf = flow->getTensorInfo("confidence").value();
    REQUIRE(fwd.dims == std::vector<unsigned int>{height, width, 2});
    REQUIRE(fwd.strides == std::vector<unsigned int>{width * 8, 8, 4});
    REQUIRE(conf.dims == std::vector<unsigned int>{height, width, 1});
    auto data = flow->getData();
    float value;
    std::memcpy(&value, data.data() + fwd.offset, sizeof(value));
    REQUIRE(value == 1.5f);
    std::memcpy(&value, data.data() + fwd.offset + 4, sizeof(value));
    REQUIRE(value == -1.5f);
    std::memcpy(&value, data.data() + bwd.offset, sizeof(value));
    REQUIRE(value == -1.5f);
    std::memcpy(&value, data.data() + fwd.offset + 8 * (width + 3), sizeof(value));
    REQUIRE(value == 4.f);
    REQUIRE(data[conf.offset] == 255);
    REQUIRE(data[conf.offset + width + 3] == 0);
    REQUIRE(output->getSequenceNum() == 103);
    REQUIRE(flow->getSequenceNum() == 103);
    auto ref = output->get<dai::Buffer>("reference");
    auto cur = output->get<dai::Buffer>("target");
    REQUIRE(ref->getSequenceNum() == 100);
    REQUIRE(cur->getSequenceNum() == 103);
    REQUIRE(cur->getTimestampDevice() - ref->getTimestampDevice() == std::chrono::nanoseconds(100000001));
    REQUIRE(ref->getData().empty());
    REQUIRE(cur->getData().empty());
}

TEST_CASE("Native optical flow rejects missing buffers and mismatched dimensions", "[OpticalFlow]") {
    dai::ImgFrame ref, target;
    ref.setWidth(128).setHeight(32);
    target.setWidth(128).setHeight(32);
    std::vector<int16_t> mv(128 * 32 * 2);
    std::vector<uint8_t> confidence(128 * 32);
    REQUIRE_THROWS_AS(dai::gate::makeOpticalFlowOutput(nullptr, mv.data(), confidence.data(), ref, target), std::invalid_argument);
    REQUIRE_THROWS_AS(dai::gate::makeOpticalFlowOutput(mv.data(), nullptr, confidence.data(), ref, target), std::invalid_argument);
    REQUIRE_THROWS_AS(dai::gate::makeOpticalFlowOutput(mv.data(), mv.data(), nullptr, ref, target), std::invalid_argument);
    ref.setWidth(256);
    REQUIRE_THROWS_AS(dai::gate::makeOpticalFlowOutput(mv.data(), mv.data(), confidence.data(), ref, target), std::invalid_argument);
}

namespace {
void setPackedImage(dai::ImgFrame& frame, unsigned int width, unsigned int height, dai::ImgFrame::Type type = dai::ImgFrame::Type::GRAY8) {
    frame.setWidth(width).setHeight(height).setType(type).setStride(0);
    frame.fb.p1Offset = 0;
    frame.setData(std::vector<uint8_t>(static_cast<std::size_t>(width) * height, 127));
}
}  // namespace

TEST_CASE("Dedicated optical flow accepts packed images without changing storage", "[OpticalFlow]") {
    dai::ImgFrame previous, current;
    setPackedImage(previous, 640, 384);
    setPackedImage(current, 640, 384);
    const auto storage = current.data;
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, &previous));
    current.setStride(640);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, &previous));
    REQUIRE(current.data == storage);
    REQUIRE(current.getData()[0] == 127);
    // Buffer pools may reserve extra bytes after the packed image.
    current.setData(std::vector<uint8_t>(640 * 384 + 4096));
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, &previous));
    setPackedImage(current, 128, 128, dai::ImgFrame::Type::RAW8);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
    setPackedImage(current, 1920, 1056);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
}

TEST_CASE("Dedicated optical flow rejects geometry outside the accelerator contract", "[OpticalFlow]") {
    dai::ImgFrame current;
    // Provide a full maximum-size payload so geometry failures cannot be hidden
    // by the separate payload-size guard. In particular, aligned 32/64/96-line
    // inputs are smaller than the accelerator's minimum height.
    setPackedImage(current, 1920, 1056);
    for(const auto width : {0U, 127U, 129U, 639U, 1921U, 2048U, std::numeric_limits<unsigned int>::max()}) {
        INFO("width=" << width);
        current.setWidth(width);
        REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    }
    current.setWidth(640);
    for(const auto height : {0U, 32U, 64U, 96U, 127U, 129U, 1080U, 1088U, std::numeric_limits<unsigned int>::max()}) {
        INFO("height=" << height);
        current.setHeight(height);
        REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    }
    current.setHeight(384).setType(dai::ImgFrame::Type::NV12);
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
}

TEST_CASE("Dedicated optical flow rejects layouts the accelerator cannot interpret", "[OpticalFlow]") {
    dai::ImgFrame current;
    setPackedImage(current, 640, 384);
    // Even a sufficiently large padded allocation must not be treated as packed.
    current.setData(std::vector<uint8_t>(768 * 384));
    current.setStride(768);
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    current.setStride(639);
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    current.setStride(640);
    current.fb.p1Offset = 1;
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    current.fb.p1Offset = 0;
    current.setData(std::vector<uint8_t>(640 * 384 - 1));
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    current.setData(std::vector<uint8_t>{});
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    setPackedImage(current, 640, 384);
    auto offsetMemory = std::make_shared<dai::ConvertedOffsetMemory>(current.data);
    offsetMemory->setOffset(1);
    current.data = offsetMemory;
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
    current.data.reset();
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, nullptr), std::invalid_argument);
}

TEST_CASE("Dedicated optical flow rejects incompatible pairs despite individually valid images", "[OpticalFlow]") {
    dai::ImgFrame previous, current;
    setPackedImage(previous, 640, 384);
    setPackedImage(current, 1280, 384);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, &previous), std::invalid_argument);
    setPackedImage(current, 640, 416);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, &previous), std::invalid_argument);
    setPackedImage(current, 640, 384, dai::ImgFrame::Type::RAW8);
    REQUIRE_NOTHROW(dai::gate::validateOpticalFlowInput(current, nullptr));
    REQUIRE_THROWS_AS(dai::gate::validateOpticalFlowInput(current, &previous), std::invalid_argument);
}

TEST_CASE("Dedicated optical flow properties survive wire serialization and cloning", "[OpticalFlow]") {
    dai::OpticalFlowProperties properties;
    std::vector<std::uint8_t> bytes;
    properties.serialize(bytes, dai::SerializationType::LIBNOP);
    dai::OpticalFlowProperties restored;
    dai::utility::deserialize(bytes, restored);
    REQUIRE(restored.reserved == 0);
    auto cloned = properties.clone();
    REQUIRE(dynamic_cast<dai::OpticalFlowProperties*>(cloned.get()) != nullptr);
}
