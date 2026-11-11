#pragma once

#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace buffer_test {

using SteadyTime = std::chrono::steady_clock::time_point;

inline const std::vector<std::uint8_t> PAYLOAD{0, 1, 127, 128, 254, 255};
inline const SteadyTime HOST_TIME{std::chrono::nanoseconds(123456789)};
inline const SteadyTime DEVICE_TIME{std::chrono::nanoseconds(234567890)};

inline std::shared_ptr<dai::Buffer> makeBuffer(std::int64_t sequenceNum) {
    auto buffer = std::make_shared<dai::Buffer>();
    buffer->setData(PAYLOAD);
    buffer->setSequenceNum(sequenceNum);
    buffer->setTimestamp(HOST_TIME);
    buffer->setTimestampDevice(DEVICE_TIME);
    return buffer;
}

inline void requireBuffer(const std::shared_ptr<dai::Buffer>& buffer, std::int64_t sequenceNum, SteadyTime hostTime = HOST_TIME) {
    REQUIRE(buffer != nullptr);
    REQUIRE(std::vector<std::uint8_t>(buffer->getData().begin(), buffer->getData().end()) == PAYLOAD);
    REQUIRE(buffer->getSequenceNum() == sequenceNum);
    REQUIRE(buffer->getTimestamp() == hostTime);
    REQUIRE(buffer->getTimestampDevice() == DEVICE_TIME);
}

inline const std::vector<std::uint8_t> IMAGE_PIXELS{0, 17, 34, 51, 68, 85, 102, 255};

inline std::shared_ptr<dai::ImgFrame> makeImage() {
    auto image = std::make_shared<dai::ImgFrame>();
    image->setSize(4, 2);
    image->setSourceSize(8, 4);
    image->setType(dai::ImgFrame::Type::GRAY8);
    image->setStride(4);
    image->setData(IMAGE_PIXELS);
    image->setSequenceNum(42);
    image->setInstanceNum(7);
    image->setCategory(3);
    image->cam.exposureTimeUs = 1234;
    image->cam.sensitivityIso = 321;
    image->cam.lensPosition = 12;
    return image;
}

// Reads the pixels through the frame's stride: a device may store received rows padded to its own alignment
inline void requireImage(const std::shared_ptr<dai::ImgFrame>& image) {
    REQUIRE(image != nullptr);
    REQUIRE(image->getType() == dai::ImgFrame::Type::GRAY8);
    REQUIRE(image->getWidth() == 4);
    REQUIRE(image->getHeight() == 2);
    REQUIRE(image->getStride() >= 4);
    REQUIRE(image->getData().size() >= image->getStride() + 4);
    std::vector<std::uint8_t> pixels;
    for(unsigned int row = 0; row < 2; ++row) {
        auto begin = image->getData().begin() + row * image->getStride();
        pixels.insert(pixels.end(), begin, begin + 4);
    }
    REQUIRE(pixels == IMAGE_PIXELS);
    REQUIRE(image->getSourceWidth() == 8);
    REQUIRE(image->getSourceHeight() == 4);
    REQUIRE(image->getSequenceNum() == 42);
    REQUIRE(image->getInstanceNum() == 7);
    REQUIRE(image->getCategory() == 3);
    REQUIRE(image->cam.exposureTimeUs == 1234);
    REQUIRE(image->cam.sensitivityIso == 321);
    REQUIRE(image->cam.lensPosition == 12);
}

template <typename T>
std::shared_ptr<T> take(const std::shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto message = queue->get<T>(std::chrono::seconds(5), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(message != nullptr);
    return message;
}

inline void requireNoMessage(const std::shared_ptr<dai::MessageQueue>& queue) {
    bool timedOut = false;
    auto message = queue->get(std::chrono::milliseconds(500), timedOut);
    REQUIRE(timedOut);
    REQUIRE(message == nullptr);
}

}  // namespace buffer_test
