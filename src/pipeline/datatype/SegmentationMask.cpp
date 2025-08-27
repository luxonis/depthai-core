#define _USE_MATH_DEFINES

#include "depthai/pipeline/datatype/SegmentationMask.hpp"

#include "depthai/utility/SharedMemory.hpp"
namespace dai {

SegmentationMask::SegmentationMask() {
    // Set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}

SegmentationMask::SegmentationMask(size_t width, size_t height) : SegmentationMask() {
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(width * height);
    data = mem;
    this->width = width;
    this->height = height;
}

SegmentationMask::SegmentationMask(long fd, size_t width, size_t height) : SegmentationMask() {
    auto mem = std::make_shared<SharedMemory>(fd, width * height);
    data = mem;
    this->width = width;
    this->height = height;
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SegmentationMask::getTimestamp() const {
    return getTimestamp();
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SegmentationMask::getTimestampDevice() const {
    return getTimestampDevice();
}

SegmentationMask& SegmentationMask::setWidth(std::size_t width) {
    this->width = width;
    return *this;
}

size_t SegmentationMask::getWidth() const {
    return width;
}

SegmentationMask& SegmentationMask::setHeight(std::size_t height) {
    this->height = height;
    return *this;
}

size_t SegmentationMask::getHeight() const {
    return height;
}

}  // namespace dai
