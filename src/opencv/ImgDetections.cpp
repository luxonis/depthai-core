#include "depthai/pipeline/datatype/ImgDetections.hpp"

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace dai {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch-enum"

ImgDetections& ImgDetections::setSegmentationMask(cv::Mat1b mask) {
    std::vector<uint8_t> dataVec;
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

cv::Mat1b ImgDetections::getSegmentationMask(bool deepCopy) {
    // Convert to cv::Mat. If deepCopy enabled, then copy pixel data, otherwise reference only
    cv::Mat mat;
    cv::Size size = cv::Size(getSegmentationMaskWidth(), getSegmentationMaskHeight());
    int type = CV_8UC1;

    // Check if enough data
    long requiredSize = CV_ELEM_SIZE(type) * size.area();

    long actualSize = static_cast<long>(data->getSize());

    if(actualSize < requiredSize) {
        throw std::runtime_error("Segmentation mask doesn't have enough data to encode specified frame, required " + std::to_string(requiredSize) + ", actual "
                                 + std::to_string(actualSize) + ". Maybe metadataOnly transfer was made?");
    }

    if(getSegmentationMaskWidth() <= 0 || getSegmentationMaskHeight() <= 0) {
        throw std::runtime_error("Segmentation mask metadata not valid (width or height <= 0).");
    }

    // Copy or reference to existing data
    if(deepCopy) {
        // Create new image data
        mat.create(size, type);
        std::memcpy(mat.data, data->getData().data(), std::min((long)(data->getSize()), (long)(mat.dataend - mat.datastart)));
    } else {
        mat = cv::Mat(size, type, data->getData().data());
    }
    cv::Mat1b mat1b = mat;
    return mat1b;
}

cv::Mat1b ImgDetections::getCvSegmentationMask(cv::MatAllocator* allocator) {
    cv::Mat1b mask = getSegmentationMask();
    cv::Mat1b output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    mask.copyTo(output);
    return output;
}

cv::Mat1b ImgDetections::getCvSegmentationMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    cv::Mat1b mask = getCvSegmentationMask(allocator);
    cv::Mat1b classMask;
    cv::compare(mask, index, classMask, cv::CmpTypes::CMP_EQ);

    return classMask;
}

#pragma GCC diagnostic pop

}  // namespace dai