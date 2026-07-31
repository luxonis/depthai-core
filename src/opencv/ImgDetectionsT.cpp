#include "depthai/pipeline/datatype/ImgDetectionsT.hpp"

#include <stdexcept>
#include <utility>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {

template <class DetectionT>
void ImgDetectionsT<DetectionT>::setCvSegmentationMask(cv::Mat mask) {
    if(mask.type() != CV_8UC1) {
        throw std::runtime_error("SetCvSegmentationMask: Mask must be of INT8 type, got opencv type " + cv::typeToString(mask.type()) + ".");
    }
    std::vector<std::uint8_t> dataVec;
    if(!mask.isContinuous()) {
        for(int i = 0; i < mask.rows; i++) {
            dataVec.insert(dataVec.end(), mask.ptr(i), mask.ptr(i) + mask.cols * mask.elemSize());
        }
    } else {
        dataVec.insert(dataVec.begin(), mask.datastart, mask.dataend);
    }
    setData(std::move(dataVec));
    this->segmentationMaskWidth = mask.cols;
    this->segmentationMaskHeight = mask.rows;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMask(cv::MatAllocator* allocator) {
    if(data->getData().data() == nullptr) return std::nullopt;

    cv::Size size(getSegmentationMaskWidth(), getSegmentationMaskHeight());
    constexpr int type = CV_8UC1;
    if(size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Segmentation mask metadata not valid (width or height <= 0).");
    }

    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();
    if(actualSize != requiredSize) {
        throw std::runtime_error("Segmentation mask data size does not match the expected size, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ".");
    }

    cv::Mat mask(size, type, data->getData().data());
    cv::Mat output;
    if(allocator != nullptr) output.allocator = allocator;
    mask.copyTo(output);
    return output;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    auto mask = getCvSegmentationMask(allocator);
    if(!mask) return std::nullopt;

    cv::Mat classMask;
    cv::compare(*mask, index, classMask, cv::CmpTypes::CMP_EQ);
    return classMask;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMaskByClass(uint8_t semanticClass, cv::MatAllocator* allocator) {
    auto mask = getCvSegmentationMask(allocator);
    if(!mask) return std::nullopt;

    cv::Mat classMask = cv::Mat::zeros(mask->size(), CV_8UC1) + 255;
    for(uint8_t idx = 0; idx < detections.size(); idx++) {
        if(detections[idx].label == semanticClass) {
            auto indexMask = getCvSegmentationMaskByIndex(idx, allocator);
            if(!indexMask) return std::nullopt;
            classMask.setTo(0, *indexMask);
        }
    }
    return classMask;
}

template void ImgDetectionsT<ImgDetection>::setCvSegmentationMask(cv::Mat);
template std::optional<cv::Mat> ImgDetectionsT<ImgDetection>::getCvSegmentationMask(cv::MatAllocator*);
template std::optional<cv::Mat> ImgDetectionsT<ImgDetection>::getCvSegmentationMaskByIndex(uint8_t, cv::MatAllocator*);
template std::optional<cv::Mat> ImgDetectionsT<ImgDetection>::getCvSegmentationMaskByClass(uint8_t, cv::MatAllocator*);
template void ImgDetectionsT<SpatialImgDetection>::setCvSegmentationMask(cv::Mat);
template std::optional<cv::Mat> ImgDetectionsT<SpatialImgDetection>::getCvSegmentationMask(cv::MatAllocator*);
template std::optional<cv::Mat> ImgDetectionsT<SpatialImgDetection>::getCvSegmentationMaskByIndex(uint8_t, cv::MatAllocator*);
template std::optional<cv::Mat> ImgDetectionsT<SpatialImgDetection>::getCvSegmentationMaskByClass(uint8_t, cv::MatAllocator*);

}  // namespace dai
