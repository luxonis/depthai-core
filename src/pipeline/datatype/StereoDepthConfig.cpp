#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {

std::shared_ptr<RawBuffer> StereoDepthConfig::serialize() const {
    return raw;
}

StereoDepthConfig::StereoDepthConfig() : Buffer(std::make_shared<RawStereoDepthConfig>()), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}
StereoDepthConfig::StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr)
    : Buffer(std::move(ptr)), cfg(*dynamic_cast<RawStereoDepthConfig*>(raw.get())) {}

void StereoDepthConfig::setConfidenceThreshold(int confThr) {
    cfg.config.confidenceThreshold = confThr;
}

int StereoDepthConfig::getConfidenceThreshold() const {
    return cfg.config.confidenceThreshold;
}

void StereoDepthConfig::setMedianFilter(dai::MedianFilter median) {
    cfg.config.median = median;
}

dai::MedianFilter StereoDepthConfig::getMedianFilter() const {
    return cfg.config.median;
}

void StereoDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    cfg.config.bilateralSigmaValue = sigma;
}

uint16_t StereoDepthConfig::getBilateralFilterSigma() const {
    return cfg.config.bilateralSigmaValue;
}

void StereoDepthConfig::setLeftRightCheckThreshold(int threshold) {
    cfg.config.leftRightCheckThreshold = threshold;
}

int StereoDepthConfig::getLeftRightCheckThreshold() const {
    return cfg.config.leftRightCheckThreshold;
}

}  // namespace dai
