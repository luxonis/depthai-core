#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> StereoDepthConfig::serialize() const {
    return raw;
}

StereoDepthConfig::StereoDepthConfig() : Buffer(std::make_shared<RawStereoDepthConfig>()), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}
StereoDepthConfig::StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}

void StereoDepthConfig::setConfidenceThreshold(int confThr) {
    cfg.costMatching.confidenceThreshold = confThr;
}

int StereoDepthConfig::getConfidenceThreshold() const {
    return cfg.costMatching.confidenceThreshold;
}

void StereoDepthConfig::setMedianFilter(dai::MedianFilter median) {
    cfg.postProcessing.median = median;
}

dai::MedianFilter StereoDepthConfig::getMedianFilter() const {
    return cfg.postProcessing.median;
}

void StereoDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    cfg.postProcessing.bilateralSigmaValue = sigma;
}

uint16_t StereoDepthConfig::getBilateralFilterSigma() const {
    return cfg.postProcessing.bilateralSigmaValue;
}

void StereoDepthConfig::setLeftRightCheckThreshold(int threshold) {
    cfg.algorithmControl.leftRightCheckThreshold = threshold;
}

int StereoDepthConfig::getLeftRightCheckThreshold() const {
    return cfg.algorithmControl.leftRightCheckThreshold;
}

void StereoDepthConfig::setLeftRightCheck(bool enable) {
    cfg.algorithmControl.enableLeftRightCheck = enable;
}

void StereoDepthConfig::setSubpixel(bool enable) {
    cfg.algorithmControl.enableSubpixel = enable;
}

dai::RawStereoDepthConfig StereoDepthConfig::get() const {
    return cfg;
}

void StereoDepthConfig::set(dai::RawStereoDepthConfig config) {
    cfg = config;
}

}  // namespace dai
