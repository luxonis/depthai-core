#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {
StereoDepthConfig& StereoDepthConfig::setDepthAlign(AlgorithmControl::DepthAlign align) {
    algorithmControl.depthAlign = align;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setConfidenceThreshold(int confThr) {
    costMatching.confidenceThreshold = confThr;
    return *this;
}

int StereoDepthConfig::getConfidenceThreshold() const {
    return costMatching.confidenceThreshold;
}

StereoDepthConfig& StereoDepthConfig::setMedianFilter(MedianFilter median) {
    postProcessing.median = median;
    return *this;
}

StereoDepthConfig::MedianFilter StereoDepthConfig::getMedianFilter() const {
    return postProcessing.median;
}

StereoDepthConfig& StereoDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    postProcessing.bilateralSigmaValue = sigma;
    return *this;
}

uint16_t StereoDepthConfig::getBilateralFilterSigma() const {
    return postProcessing.bilateralSigmaValue;
}

StereoDepthConfig& StereoDepthConfig::setLeftRightCheckThreshold(int threshold) {
    algorithmControl.leftRightCheckThreshold = threshold;
    return *this;
}

int StereoDepthConfig::getLeftRightCheckThreshold() const {
    return algorithmControl.leftRightCheckThreshold;
}

StereoDepthConfig& StereoDepthConfig::setLeftRightCheck(bool enable) {
    algorithmControl.enableLeftRightCheck = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setExtendedDisparity(bool enable) {
    algorithmControl.enableExtended = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setSubpixel(bool enable) {
    algorithmControl.enableSubpixel = enable;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setSubpixelFractionalBits(int subpixelFractionalBits) {
    algorithmControl.subpixelFractionalBits = subpixelFractionalBits;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    algorithmControl.depthUnit = depthUnit;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setDisparityShift(int disparityShift) {
    algorithmControl.disparityShift = disparityShift;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels) {
    algorithmControl.numInvalidateEdgePixels = numInvalidateEdgePixels;
    return *this;
}

dai::StereoDepthConfig::AlgorithmControl::DepthUnit StereoDepthConfig::getDepthUnit() {
    return algorithmControl.depthUnit;
}

float StereoDepthConfig::getMaxDisparity() const {
    float maxDisp = 95.0;
    if(costMatching.disparityWidth == StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64) {
        maxDisp = 63;
    }
    if(costMatching.enableCompanding) maxDisp = 175;
    maxDisp += algorithmControl.disparityShift;
    if(algorithmControl.enableExtended) maxDisp *= 2;
    if(algorithmControl.enableSubpixel) maxDisp *= (1 << algorithmControl.subpixelFractionalBits);
    return maxDisp;
}
}  // namespace dai
