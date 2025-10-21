#include <cstddef>
#include <cstdint>
#include <vector>
#define _USE_MATH_DEFINES

#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/utility/SharedMemory.hpp"

#ifdef DEPTHAI_XTENSOR_SUPPORT
    #include <xtensor/containers/xadapt.hpp>
    #include <xtensor/core/xtensor_forward.hpp>
    #include <xtensor/views/xstrided_view.hpp>
    #include <xtensor/core/xmath.hpp>
#endif
namespace dai {

SegmentationMask::SegmentationMask() {
    // Set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}

SegmentationMask::SegmentationMask(size_t width, size_t height) : SegmentationMask() {
    if(width <= 0 || height <= 0) {
        throw std::runtime_error("SegmentationMask: width and height must be greater than 0.");
    }
    auto mem = std::make_shared<VectorMemory>();
    mem->resize(width * height);
    data = mem;
    this->width = width;
    this->height = height;
}

SegmentationMask::SegmentationMask(std::vector<int> mask, size_t width, size_t height) : SegmentationMask() {
    if(width <= 0 || height <= 0) {
        throw std::runtime_error("SegmentationMask: width and height must be greater than 0.");
    }
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height.");
    }
    // check all values are in range 0-255
    for(auto& v : mask) {
        if(v < 0 || v > 255) {
            throw std::runtime_error("SegmentationMask: data values must be in range 0- 255.");
        }
    }
    std::vector<std::uint8_t> maskU8(mask.begin(), mask.end());
    setData(maskU8);
    this->width = width;
    this->height = height;
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SegmentationMask::getTimestamp() const {
    return Buffer::getTimestamp();
}

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> SegmentationMask::getTimestampDevice() const {
    return Buffer::getTimestampDevice();
}

size_t SegmentationMask::getWidth() const {
    return width;
}

size_t SegmentationMask::getHeight() const {
    return height;
}

// Optional - xtensor support
#ifdef DEPTHAI_XTENSOR_SUPPORT

SegmentationMask::SegmentationMask(xt::xarray<uint8_t>& mask) : SegmentationMask() {
    data->setSize(mask.size());
    std::vector<uint8_t> dataVec(mask.begin(), mask.end());
    setData(dataVec);

    this->width = static_cast<size_t>(mask.shape()[1]);
    this->height = static_cast<size_t>(mask.shape()[0]);
}

xt::xarray<uint8_t> SegmentationMask::getTensorMask() const {
    size_t dataSize = data->getSize();
    if(dataSize != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }

    std::array<std::size_t, 2> shape{height, width};
    return xt::adapt(data->getData().data(), data->getSize(), xt::no_ownership(), shape);
}

SegmentationMask& SegmentationMask::setTensorMask(xt::xarray<uint8_t>& mask) {
    if(mask.shape()[0] != height || mask.shape()[1] != width) {
        throw std::runtime_error("SegmentationMask: input mask shape does not match current width and height");
    }
    data->setSize(mask.size());
    std::vector<uint8_t> dataVec(mask.begin(), mask.end());
    setData(dataVec);
    return *this;
}

xt::xarray<uint8_t> SegmentationMask::getTensorMaskByIndex(std::uint8_t index) const {
    const auto& buf = data->getData();
    if(buf.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    std::vector<size_t> shape{height, width};
    auto in = xt::adapt(buf.data(), buf.size(), xt::no_ownership(), shape);

    return xt::eval(xt::cast<std::uint8_t>(xt::equal(in, index))), xt::xarray<std::uint8_t>();
}

#endif

}  // namespace dai
