#include "depthai/pipeline/datatype/VppConfig.hpp"

namespace dai {

VppConfig& VppConfig::setMethod(Method method) {
    this->method = method;
    return *this;
}

VppConfig& VppConfig::setPatchSize(int size) {
    this->patchSize = size;
    return *this;
}

VppConfig& VppConfig::setAggregationWindowSize(int sizeX, int sizeY) {
    this->windowSizeAggX = sizeX;
    this->windowSizeAggY = sizeY;
    return *this;
}

VppConfig& VppConfig::setProjectionDirection(bool left2right) {
    this->left2right = left2right;
    return *this;
}

VppConfig& VppConfig::setBlending(float blending, float blendingOcclusion) {
    this->blending = blending;
    this->blendingOcclusion = blendingOcclusion;
    return *this;
}

VppConfig& VppConfig::setDistancePatch(bool enable, float gamma) {
    this->useDistancePatch = enable;
    this->distanceGamma = gamma;
    return *this;
}

VppConfig& VppConfig::setBilateralPatch(bool enable, float spatialSigma, float intensitySigma, float threshold) {
    this->useBilateralPatch = enable;
    this->bilateralSpatialSigma = spatialSigma;
    this->bilateralIntensitySigma = intensitySigma;
    this->bilateralThreshold = threshold;
    return *this;
}

VppConfig& VppConfig::setDisparityRange(float minThreshold, float maxThreshold) {
    this->disparityMinThreshold = minThreshold;
    this->disparityMaxThreshold = maxThreshold;
    return *this;
}

}  // namespace dai