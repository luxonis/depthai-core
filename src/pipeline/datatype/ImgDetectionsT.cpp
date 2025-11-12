#include "depthai/pipeline/datatype/ImgDetectionsT.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
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
void ImgDetectionsT<DetectionT>::setMask(const std::vector<std::uint8_t>& mask, size_t width, size_t height) {
    if(mask.size() != width * height) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    setData(mask);
    this->segmentationMaskWidth = width;
    this->segmentationMaskHeight = height;
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
std::optional<dai::ImgFrame> ImgDetectionsT<DetectionT>::getSegmentationMaskAsImgFrame() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }
    dai::ImgFrame img;
    img.setWidth(segmentationMaskWidth);
    img.setHeight(segmentationMaskHeight);
    img.setType(dai::ImgFrame::Type::GRAY8);
    img.setSequenceNum(sequenceNum);
    img.setTimestamp(getTimestamp());
    img.setTimestampDevice(getTimestampDevice());
    img.setData(*maskData);

    return img;
}

// Optional - xtensor support
#ifdef DEPTHAI_XTENSOR_SUPPORT

template <class DetectionT>
std::optional<typename ImgDetectionsT<DetectionT>::XArray2D> ImgDetectionsT<DetectionT>::getTensorSegmentationMask() const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    size_t dataSize = (*maskData).size();
    if(dataSize != segmentationMaskWidth * segmentationMaskHeight) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }

    using XArray2D = typename ImgDetectionsT<DetectionT>::XArray2D;
    std::array<std::size_t, 2> shape{segmentationMaskHeight, segmentationMaskWidth};
    auto result = XArray2D::from_shape(shape);
    std::copy(maskData->cbegin(), maskData->cend(), result.begin());
    return result;
}

template <class DetectionT>
ImgDetectionsT<DetectionT>& ImgDetectionsT<DetectionT>::setTensorSegmentationMask(typename ImgDetectionsT<DetectionT>::XArray2D mask) {
    data->setSize(mask.size());
    std::vector<uint8_t> dataVec(mask.begin(), mask.end());
    setData(dataVec);
    this->segmentationMaskWidth = mask.shape()[1];
    this->segmentationMaskHeight = mask.shape()[0];
    return *this;
}

template <class DetectionT>
std::optional<typename ImgDetectionsT<DetectionT>::XArray2D> ImgDetectionsT<DetectionT>::getTensorSegmentationMaskByIndex(std::uint8_t index) const {
    std::optional<std::vector<std::uint8_t>> maskData = getMaskData();
    if(!maskData) {
        return std::nullopt;
    }

    if((*maskData).size() != segmentationMaskWidth * segmentationMaskHeight) {
        throw std::runtime_error("SegmentationMask: data size does not match width*height");
    }
    using XArray2D = typename ImgDetectionsT<DetectionT>::XArray2D;
    std::array<std::size_t, 2> shape{segmentationMaskHeight, segmentationMaskWidth};
    auto result = XArray2D::from_shape(shape);
    auto dstIt = result.begin();
    auto srcIt = maskData->cbegin();
    for(; dstIt != result.end(); ++dstIt, ++srcIt) {
        *dstIt = static_cast<std::uint8_t>(*srcIt == index);
    }

    return result;
}

#endif

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

template <class DetectionT>
ImgDetectionsT<DetectionT>& ImgDetectionsT<DetectionT>::setSegmentationMask(cv::Mat mask) {
    std::vector<std::uint8_t> dataVec;
    if(!mask.isContinuous()) {
        for(int i = 0; i < mask.rows; i++) {
            dataVec.insert(dataVec.end(), mask.ptr(i), mask.ptr(i) + mask.cols * mask.elemSize());
        }
    } else {
        dataVec.insert(dataVec.begin(), mask.datastart, mask.dataend);
    }
    setData(dataVec);
    this->segmentationMaskWidth = mask.cols;
    this->segmentationMaskHeight = mask.rows;
    return *this;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getSegmentationMask(bool deepCopy) {
    if(data->getData().data() == nullptr) {
        return std::nullopt;
    }
    cv::Size size(getSegmentationMaskWidth(), getSegmentationMaskHeight());
    int type = CV_8UC1;
    if(size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Segmentation mask metadata not valid (width or height <= 0).");
    }

    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();

    if(actualSize != requiredSize) {
        throw std::runtime_error("Segmentation mask data size does not match the expected size, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ".");
    }

    cv::Mat mat;
    if(deepCopy) {
        mat.create(size, type);
        std::memcpy(mat.data, data->getData().data(), std::min((long)(data->getSize()), (long)(mat.dataend - mat.datastart)));
    } else {
        mat = cv::Mat(size, type, data->getData().data());
    }
    CV_Assert(mat.type() == CV_8UC1);

    return mat;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMask(cv::MatAllocator* allocator) {
    std::optional<cv::Mat> mask = getSegmentationMask();
    if(!mask.has_value()) {
        return std::nullopt;
    }
    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    (*mask).copyTo(output);
    return output;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    std::optional<cv::Mat> mask = getCvSegmentationMask(allocator);
    if(!mask.has_value()) {
        return std::nullopt;
    }
    cv::Mat classMask;
    cv::compare(*mask, index, classMask, cv::CmpTypes::CMP_EQ);

    return classMask;
}

template <class DetectionT>
std::optional<cv::Mat> ImgDetectionsT<DetectionT>::getCvSegmentationMaskByClass(uint8_t semanticClass, cv::MatAllocator* allocator) {
    std::optional<cv::Mat> mask = getCvSegmentationMask(allocator);
    if(!mask.has_value()) {
        return std::nullopt;
    }
    cv::Mat classMask = cv::Mat::zeros((*mask).size(), CV_8UC1) + 255;

    for(uint8_t idx = 0; idx < detections.size(); idx++) {
        if(detections[idx].label == semanticClass) {
            std::optional<cv::Mat> indexMask = getCvSegmentationMaskByIndex(idx, allocator);
            if(!indexMask.has_value()) {
                return std::nullopt;
            }
            classMask.setTo(0, *indexMask);
        }
    }

    return classMask;
}

#endif

template class ImgDetectionsT<dai::ImgDetection>;
template class ImgDetectionsT<dai::SpatialImgDetection>;

}  // namespace dai
