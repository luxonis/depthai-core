#include "depthai/pipeline/datatype/SegmentationMask.hpp"

#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "utility/ErrorMacros.hpp"

namespace dai {

void SegmentationMask::setCvMask(cv::Mat mask) {
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
    width = mask.cols;
    height = mask.rows;
}

cv::Mat SegmentationMask::getCvMask(cv::MatAllocator* allocator) {
    cv::Mat mask;
    if(data->getData().data() == nullptr || data->getSize() == 0) {
        return mask;
    }
    cv::Size size(static_cast<int>(getWidth()), static_cast<int>(getHeight()));
    constexpr int type = CV_8UC1;

    const size_t requiredSize = CV_ELEM_SIZE(type) * static_cast<size_t>(size.area());
    const size_t actualSize = data->getSize();
    DAI_CHECK_V(actualSize == requiredSize, "Segmentation mask data size does not match the expected size, required {}, actual {}.", requiredSize, actualSize);

    mask = cv::Mat(size, type, data->getData().data());
    cv::Mat output;
    if(allocator != nullptr) {
        output.allocator = allocator;
    }
    mask.copyTo(output);
    return output;
}

cv::Mat SegmentationMask::getCvMaskByIndex(uint8_t index, cv::MatAllocator* allocator) {
    cv::Mat mask = getCvMask(allocator);
    if(mask.empty()) {
        return {};
    }

    cv::Mat indexedMask;
    cv::compare(mask, index, indexedMask, cv::CmpTypes::CMP_EQ);
    return indexedMask;
}

std::vector<std::vector<dai::Point2f>> SegmentationMask::getContour(uint8_t index) {
    std::vector<std::vector<dai::Point2f>> result;
    cv::Mat mask = getCvMaskByIndex(index);
    if(mask.empty()) {
        return result;
    }
    cv::Mat maskCopy = mask.clone();
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(maskCopy, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
    for(const auto& contour : contours) {
        std::vector<dai::Point2f> daiContour;
        for(const auto& point : contour) {
            daiContour.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y), false);
        }
        result.emplace_back(std::move(daiContour));
    }

    return result;
}

std::vector<dai::RotatedRect> SegmentationMask::getBoundingBoxes(uint8_t index, bool calculateRotation) {
    std::vector<dai::RotatedRect> boxes;
    cv::Mat mask = getCvMaskByIndex(index);
    if(mask.empty()) {
        return {};
    }

    cv::Mat maskCopy = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(maskCopy, contours, cv::RetrievalModes::RETR_EXTERNAL, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);
    if(contours.empty()) {
        return {};
    }
    const float widthF = static_cast<float>(width);
    const float heightF = static_cast<float>(height);

    for(const auto& contour : contours) {
        dai::RotatedRect box;
        if(calculateRotation) {
            cv::RotatedRect cvBox = cv::minAreaRect(contour);
            box = {dai::Point2f(cvBox.center.x / widthF, cvBox.center.y / heightF, true),
                   dai::Size2f(cvBox.size.width / widthF, cvBox.size.height / heightF, true),
                   cvBox.angle};
        } else {
            cv::Rect boundingRect = cv::boundingRect(contour);
            if(boundingRect.width == 0 || boundingRect.height == 0) {
                continue;
            }
            box = {dai::Point2f((boundingRect.x + boundingRect.width / 2.0f) / widthF, (boundingRect.y + boundingRect.height / 2.0f) / heightF, true),
                   dai::Size2f(boundingRect.width / widthF, boundingRect.height / heightF, true),
                   0.0f};
        }

        boxes.push_back(box);
    }

    return boxes;
}

}  // namespace dai
