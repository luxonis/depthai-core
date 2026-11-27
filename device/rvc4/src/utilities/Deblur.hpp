#pragma once
#include <opencv2/core.hpp>
#include <vector>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/properties/DeblurProperties.hpp"
namespace dai {
namespace gate {
namespace deblur {
struct Field {
    cv::Mat image;   // owned FP32 [0,255]
    cv::Mat labels;  // S32; -1 is passthrough
    cv::Mat blend;   // FP32 effective blend [0,1]
    std::vector<cv::Mat> kernels;
};
void validateImage(const ImgFrame& image);
cv::Mat copyImage(const ImgFrame& image);
cv::Mat linePsf(cv::Point2f vector);
cv::Mat inverseKernel(cv::Point2f vector, const DeblurProperties& config);
Field prepare(const ImgFrame& reference, const ImgFrame& target, const MessageGroup& packet, const DeblurProperties& config);
cv::Mat filterCpu(const Field& field);
cv::Mat finish(const Field& field, const cv::Mat& filtered, const DeblurProperties& config);
}  // namespace deblur
}  // namespace gate
}  // namespace dai
