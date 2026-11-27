#include "SpeckleFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <opencv2/calib3d.hpp>

namespace dai {

SpeckleFilter::SpeckleFilter() {}

int SpeckleFilter::Init() {
    return 0;
}

SpeckleFilter::~SpeckleFilter() {}

void SpeckleFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int speckleRange, int maxDiff) {
    cv::Mat cvDisparity = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), CV_16SC1, disparityFrame->data->getData().data());
    cv::filterSpeckles(cvDisparity, 0, speckleRange, maxDiff);
}

}  // namespace dai
