#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> StereoDepthConfig::serialize() const {
    return raw;
}

StereoDepthConfig::StereoDepthConfig() : Buffer(std::make_shared<RawStereoDepthConfig>()), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}
StereoDepthConfig::StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}

void StereoDepthConfig::setConfidenceThreshold(int confThr) {
    cfg.config.costMatching.confidenceThreshold = confThr;
}

int StereoDepthConfig::getConfidenceThreshold() const {
    return cfg.config.costMatching.confidenceThreshold;
}

void StereoDepthConfig::setMedianFilter(dai::MedianFilter median) {
    cfg.config.postProcessing.median = median;
}

dai::MedianFilter StereoDepthConfig::getMedianFilter() const {
    return cfg.config.postProcessing.median;
}

void StereoDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    cfg.config.postProcessing.bilateralSigmaValue = sigma;
}

uint16_t StereoDepthConfig::getBilateralFilterSigma() const {
    return cfg.config.postProcessing.bilateralSigmaValue;
}

void StereoDepthConfig::setLeftRightCheckThreshold(int threshold) {
    cfg.config.algorithmControl.leftRightCheckThreshold = threshold;
}

int StereoDepthConfig::getLeftRightCheckThreshold() const {
    return cfg.config.algorithmControl.leftRightCheckThreshold;
}

void StereoDepthConfig::setLeftRightCheck(bool enable) {
    cfg.config.algorithmControl.enableLeftRightCheck = enable;
}

void StereoDepthConfig::setSubpixel(bool enable) {
    cfg.config.algorithmControl.enableSubpixel = enable;
}

dai::RawStereoDepthConfig StereoDepthConfig::get() const {
    return cfg;
}

void StereoDepthConfig::set(dai::RawStereoDepthConfig config) {
    cfg = config;
}

}  // namespace dai
