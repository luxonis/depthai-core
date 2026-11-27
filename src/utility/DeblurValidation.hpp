#pragma once
#include <cmath>
#include <stdexcept>

#include "depthai/properties/DeblurProperties.hpp"
namespace dai {
namespace utility {
inline void validateDeblurConfig(const DeblurProperties& p) {
    const auto range = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
    if(!range(p.strength, 0, 1) || !range(p.regularization, 0.001f, 1) || !range(p.maxGain, 1, 8) || !range(p.minBlur, 0.25f, 24)
       || !range(p.maxBlur, p.minBlur, 24) || !range(p.motionQuantum, 0.25f, 4) || !range(p.forwardBackwardThreshold, 0.1f, 10)
       || !range(p.photometricThreshold, 1, 255) || !range(p.minTextureVariance, 0, 65025) || !range(p.minConfidence, 0, 255) || !range(p.maxCorrection, 0, 255)
       || p.kernelRadius < 1 || p.kernelRadius > 12 || p.maxClusters < 1 || p.maxClusters > 128 || p.exposureTimeUs > 1000000 || p.maxWaitMs > 1000
       || p.cacheSize < 2 || p.cacheSize > 32) {
        throw std::invalid_argument("Invalid Deblur configuration");
    }
}
}  // namespace utility
}  // namespace dai
