#include "MedianFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <opencv2/calib3d.hpp>

namespace dai2 {

MedianFilter::MedianFilter() {}

int MedianFilter::Init() {
    return 0;
}

MedianFilter::~MedianFilter() {}

void MedianFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int medianSize) {
    cv::Mat cvDisparity = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), CV_16UC1, disparityFrame->data->getData().data());

    if(medianSize % 2 == 0) {
        throw std::runtime_error("Median filter size must be odd");
    }
    if(medianSize > 5) {
        throw std::runtime_error("Median filter size must be <= 5");
    }

    cv::Mat cvDisparityFiltered = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), CV_16UC1);
    cv::medianBlur(cvDisparity, cvDisparityFiltered, medianSize);

    std::memcpy(disparityFrame->data->getData().data(), cvDisparityFiltered.data, cvDisparityFiltered.total() * cvDisparityFiltered.elemSize());
}

}  // namespace dai2
