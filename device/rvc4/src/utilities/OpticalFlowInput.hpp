#pragma once
#include <cstddef>
#include <stdexcept>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
namespace dai {
namespace gate {
inline void validateOpticalFlowInput(const ImgFrame& current, const ImgFrame* previous) {
    const auto width = current.getWidth();
    const auto height = current.getHeight();
    const auto type = current.getType();
    if(width < 128 || height < 128 || width > 1920 || height > 1056 || width % 128 || height % 32) {
        throw std::invalid_argument("OpticalFlow requires width 128..1920 divisible by 128 and height 128..1056 divisible by 32");
    }
    if(type != ImgFrame::Type::GRAY8 && type != ImgFrame::Type::RAW8) {
        throw std::invalid_argument("OpticalFlow requires GRAY8 or RAW8 input");
    }
    // The accelerator derives its image layout from dimensions, not ImgFrame's
    // stride or plane offset. Reject unsupported layouts before importing memory.
    if(current.getStride() != width || current.fb.p1Offset != 0) {
        throw std::invalid_argument("OpticalFlow requires packed input rows and zero first-plane offset");
    }
    // Geometry was bounded above, so this multiplication cannot overflow size_t.
    const auto requiredBytes = static_cast<std::size_t>(width) * height;
    if(!current.data || current.data->getSize() < requiredBytes) {
        throw std::invalid_argument("OpticalFlow input payload must contain at least width * height bytes");
    }
    if(current.data->getOffset() != 0) {
        throw std::invalid_argument("OpticalFlow requires zero memory offset");
    }
    if(previous && (width != previous->getWidth() || height != previous->getHeight() || type != previous->getType())) {
        throw std::invalid_argument("OpticalFlow input dimensions and format must remain constant");
    }
}
}  // namespace gate
}  // namespace dai
