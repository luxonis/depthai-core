#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {

StereoDepthConfig::~StereoDepthConfig() = default;

void StereoDepthConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::StereoDepthConfig;
}
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

bool StereoDepthConfig::getLeftRightCheck() const {
    return algorithmControl.enableLeftRightCheck;
}

StereoDepthConfig& StereoDepthConfig::setExtendedDisparity(bool enable) {
    algorithmControl.enableExtended = enable;
    return *this;
}

bool StereoDepthConfig::getExtendedDisparity() const {
    return algorithmControl.enableExtended;
}

StereoDepthConfig& StereoDepthConfig::setSubpixel(bool enable) {
    algorithmControl.enableSubpixel = enable;
    return *this;
}

bool StereoDepthConfig::getSubpixel() const {
    return algorithmControl.enableSubpixel;
}

StereoDepthConfig& StereoDepthConfig::setSubpixelFractionalBits(int subpixelFractionalBits) {
    algorithmControl.subpixelFractionalBits = subpixelFractionalBits;
    return *this;
}

int StereoDepthConfig::getSubpixelFractionalBits() const {
    return algorithmControl.subpixelFractionalBits;
}

StereoDepthConfig& StereoDepthConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    algorithmControl.depthUnit = depthUnit;
    return *this;
}

dai::StereoDepthConfig::AlgorithmControl::DepthUnit StereoDepthConfig::getDepthUnit() {
    return algorithmControl.depthUnit;
}

StereoDepthConfig& StereoDepthConfig::setCustomDepthUnitMultiplier(float multiplier) {
    algorithmControl.customDepthUnitMultiplier = multiplier;
    return *this;
}

float StereoDepthConfig::getCustomDepthUnitMultiplier() const {
    return algorithmControl.customDepthUnitMultiplier;
}

StereoDepthConfig& StereoDepthConfig::setDisparityShift(int disparityShift) {
    algorithmControl.disparityShift = disparityShift;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels) {
    algorithmControl.numInvalidateEdgePixels = numInvalidateEdgePixels;
    return *this;
}

StereoDepthConfig& StereoDepthConfig::setFiltersComputeBackend(dai::ProcessorType filtersBackend) {
    this->filtersBackend = filtersBackend;
    return *this;
}

dai::ProcessorType StereoDepthConfig::getFiltersComputeBackend() const {
    return filtersBackend;
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

    std::vector<dai::StereoDepthConfig::PostProcessing::Filter> filtersToExecute;
    for(auto filter : postProcessing.filteringOrder) {
        switch(filter) {
            case StereoDepthConfig::PostProcessing::Filter::SPECKLE:
                if(postProcessing.speckleFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case StereoDepthConfig::PostProcessing::Filter::TEMPORAL:
                if(postProcessing.temporalFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case StereoDepthConfig::PostProcessing::Filter::SPATIAL:
                if(postProcessing.spatialFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case StereoDepthConfig::PostProcessing::Filter::DECIMATION:
                if(postProcessing.decimationFilter.decimationFactor > 1) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case StereoDepthConfig::PostProcessing::Filter::MEDIAN:
                if(postProcessing.median != MedianFilter::MEDIAN_OFF) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case StereoDepthConfig::PostProcessing::Filter::NONE:
                break;
            default:
                break;
        }
    }

    if(filtersToExecute.size() != 0) {
        if(filtersToExecute.back() != StereoDepthConfig::PostProcessing::Filter::MEDIAN) {
            maxDisp = maxDisp * ((1 << 13) / maxDisp);
        }
    }

    return maxDisp;
}
}  // namespace dai
