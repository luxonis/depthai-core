#include "depthai/pipeline/datatype/ImgDetections.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace dai {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"

ImgDetections& ImgDetections::setSegmentationMask(cv::Mat mask) {
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

cv::Mat ImgDetections::getSegmentationMask(bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    if(data->getData().data() == nullptr) {
        return cv::Mat();
    }
    cv::Size size(getSegmentationMaskWidth(), getSegmentationMaskHeight());
    int type = CV_8UC1;
    if(size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Segmentation mask metadata not valid (width or height <= 0).");
    }

    // Check if enough data
    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();

    if(actualSize != requiredSize) {
        throw std::runtime_error("Segmentation mask data size does not match the expected size, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ".");
    }

    cv::Mat mat;
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);
        std::memcpy(mat.data, data->getData().data(), std::min((long)(data->getSize()), (long)(mat.dataend - mat.datastart)));
    } else {
        mat = cv::Mat(size, type, data->getData().data());
    }
    CV_Assert(mat.type() == CV_8UC1);

    return mat;
}

cv::Mat ImgDetections::getCvSegmentationMask(cv::MatAllocator* allocator) {
    cv::Mat mask = getSegmentationMask();
    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    mask.copyTo(output);
    return output;
}

cv::Mat ImgDetections::getCvSegmentationMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    cv::Mat mask = getCvSegmentationMask(allocator);
    cv::Mat classMask;
    cv::compare(mask, index, classMask, cv::CmpTypes::CMP_EQ);

    return classMask;
}

#pragma GCC diagnostic pop

}  // namespace dai