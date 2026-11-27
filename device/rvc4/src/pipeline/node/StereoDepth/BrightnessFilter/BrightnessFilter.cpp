#include "BrightnessFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dai {

BrightnessFilter::BrightnessFilter() {}

int BrightnessFilter::Init() {
    return 0;
}

BrightnessFilter::~BrightnessFilter() {}

void BrightnessFilter::process(const std::shared_ptr<const dai::ImgFrame>& referenceFrame,
                               std::shared_ptr<dai::ImgFrame>& disparityFrame,
                               int minBrightness,
                               int maxBrightness) {
    uint16_t* dispData = (uint16_t*)disparityFrame->data->getData().data();
    uint8_t* refData = (uint8_t*)referenceFrame->data->getData().data();

    // TODO(Szabolcs) - handle striding
    auto maxSize = disparityFrame->data->getData().size() / sizeof(uint16_t);
    if(referenceFrame->data->getData().size() >= maxSize) {
        for(int i = 0; i < maxSize; i++) {
            if(refData[i] < minBrightness || refData[i] > maxBrightness) {
                dispData[i] = 0;
            }
        }
    }
}

}  // namespace dai
