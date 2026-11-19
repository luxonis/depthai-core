#include "ThresholdFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dai {

ThresholdFilter::ThresholdFilter() {}

int ThresholdFilter::Init() {
    return 0;
}

ThresholdFilter::~ThresholdFilter() {}

void ThresholdFilter::process(std::shared_ptr<dai::ImgFrame>& depthFrame, int minDepth, int maxDepth) {
    if(minDepth == 0 && maxDepth == UINT16_MAX) return;
    if(maxDepth < minDepth) return;
    uint16_t* data = (uint16_t*)depthFrame->data->getData().data();

    for(int i = 0; i < depthFrame->getWidth() * depthFrame->getHeight(); i++) {
        if(data[i] < minDepth || data[i] > maxDepth) {
            data[i] = 0;
        }
    }
}

}  // namespace dai
