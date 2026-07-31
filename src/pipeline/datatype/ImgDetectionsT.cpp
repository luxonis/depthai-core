#include "depthai/pipeline/datatype/ImgDetectionsT.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>
#include <vector>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

template <class DetectionT>
size_t ImgDetectionsT<DetectionT>::getSegmentationMaskWidth() const {
    return segmentationMaskWidth;
}

template <class DetectionT>
size_t ImgDetectionsT<DetectionT>::getSegmentationMaskHeight() const {
    return segmentationMaskHeight;
}

template <class DetectionT>
void ImgDetectionsT<DetectionT>::setSegmentationMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height) {
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    auto vecMask = mask;          // Avoid mutating shared storage by moving a copy of the input into a new buffer
    setData(std::move(vecMask));  // Call the rvalue overload to allocate a new memory holder
    this->segmentationMaskWidth = width;
    this->segmentationMaskHeight = height;
}

template <class DetectionT>
void ImgDetectionsT<DetectionT>::setSegmentationMask(dai::ImgFrame& frame) {
    if(frame.getType() != dai::ImgFrame::Type::GRAY8) {
        throw std::runtime_error("SegmentationMask: ImgFrame type must be GRAY8");
    }
    auto dataSpan = frame.getData();
    std::vector<std::uint8_t> vecMask(dataSpan.begin(), dataSpan.end());
    setData(std::move(vecMask));  // Call the rvalue overload to allocate a new memory holder
    this->segmentationMaskWidth = frame.getWidth();
    this->segmentationMaskHeight = frame.getHeight();
}

template <class DetectionT>
std::optional<std::vector<std::uint8_t>> ImgDetectionsT<DetectionT>::getMaskData() const {
    const auto& d = data->getData();
    std::vector<std::uint8_t> vecMask(d.begin(), d.end());
    if(vecMask.empty()) {
        return std::nullopt;
    }
    return vecMask;
}

template <class DetectionT>
std::optional<dai::ImgFrame> ImgDetectionsT<DetectionT>::getSegmentationMask() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }
    dai::ImgFrame img;
    img.setWidth(segmentationMaskWidth);
    img.setHeight(segmentationMaskHeight);
    img.setType(dai::ImgFrame::Type::GRAY8);
    img.setBufferMetadataFrom(this);
    img.setData(*maskData);

    return img;
}

template class ImgDetectionsT<dai::ImgDetection>;
template class ImgDetectionsT<dai::SpatialImgDetection>;

}  // namespace dai
