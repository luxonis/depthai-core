#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"

namespace dai {

NeuralDepthConfig::~NeuralDepthConfig() = default;

void NeuralDepthConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::NeuralDepthConfig;
}
NeuralDepthConfig& NeuralDepthConfig::setDepthAlign(AlgorithmControl::DepthAlign align) {
    algorithmControl.depthAlign = align;
    return *this;
}

NeuralDepthConfig& NeuralDepthConfig::setConfidenceThreshold(int confThr) {
    costMatching.confidenceThreshold = confThr;
    return *this;
}

int NeuralDepthConfig::getConfidenceThreshold() const {
    return costMatching.confidenceThreshold;
}

NeuralDepthConfig& NeuralDepthConfig::setMedianFilter(MedianFilter median) {
    postProcessing.median = median;
    return *this;
}

NeuralDepthConfig::MedianFilter NeuralDepthConfig::getMedianFilter() const {
    return postProcessing.median;
}

NeuralDepthConfig& NeuralDepthConfig::setBilateralFilterSigma(uint16_t sigma) {
    postProcessing.bilateralSigmaValue = sigma;
    return *this;
}

uint16_t NeuralDepthConfig::getBilateralFilterSigma() const {
    return postProcessing.bilateralSigmaValue;
}

NeuralDepthConfig& NeuralDepthConfig::setLeftRightCheckThreshold(int threshold) {
    algorithmControl.leftRightCheckThreshold = threshold;
    return *this;
}

int NeuralDepthConfig::getLeftRightCheckThreshold() const {
    return algorithmControl.leftRightCheckThreshold;
}

NeuralDepthConfig& NeuralDepthConfig::setLeftRightCheck(bool enable) {
    algorithmControl.enableLeftRightCheck = enable;
    return *this;
}

bool NeuralDepthConfig::getLeftRightCheck() const {
    return algorithmControl.enableLeftRightCheck;
}

NeuralDepthConfig& NeuralDepthConfig::setExtendedDisparity(bool enable) {
    algorithmControl.enableExtended = enable;
    return *this;
}

bool NeuralDepthConfig::getExtendedDisparity() const {
    return algorithmControl.enableExtended;
}

NeuralDepthConfig& NeuralDepthConfig::setSubpixel(bool enable) {
    algorithmControl.enableSubpixel = enable;
    return *this;
}

bool NeuralDepthConfig::getSubpixel() const {
    return algorithmControl.enableSubpixel;
}

NeuralDepthConfig& NeuralDepthConfig::setSubpixelFractionalBits(int subpixelFractionalBits) {
    algorithmControl.subpixelFractionalBits = subpixelFractionalBits;
    return *this;
}

int NeuralDepthConfig::getSubpixelFractionalBits() const {
    return algorithmControl.subpixelFractionalBits;
}

NeuralDepthConfig& NeuralDepthConfig::setDepthUnit(AlgorithmControl::DepthUnit depthUnit) {
    algorithmControl.depthUnit = depthUnit;
    return *this;
}

dai::NeuralDepthConfig::AlgorithmControl::DepthUnit NeuralDepthConfig::getDepthUnit() {
    return algorithmControl.depthUnit;
}

NeuralDepthConfig& NeuralDepthConfig::setDisparityShift(int disparityShift) {
    algorithmControl.disparityShift = disparityShift;
    return *this;
}

NeuralDepthConfig& NeuralDepthConfig::setNumInvalidateEdgePixels(int32_t numInvalidateEdgePixels) {
    algorithmControl.numInvalidateEdgePixels = numInvalidateEdgePixels;
    return *this;
}

NeuralDepthConfig& NeuralDepthConfig::setFiltersComputeBackend(dai::ProcessorType filtersBackend) {
    this->filtersBackend = filtersBackend;
    return *this;
}

dai::ProcessorType NeuralDepthConfig::getFiltersComputeBackend() const {
    return filtersBackend;
}

float NeuralDepthConfig::getMaxDisparity() const {
    float maxDisp = 95.0;
    if(costMatching.disparityWidth == NeuralDepthConfig::CostMatching::DisparityWidth::DISPARITY_64) {
        maxDisp = 63;
    }
    if(costMatching.enableCompanding) maxDisp = 175;
    maxDisp += algorithmControl.disparityShift;
    if(algorithmControl.enableExtended) maxDisp *= 2;
    if(algorithmControl.enableSubpixel) maxDisp *= (1 << algorithmControl.subpixelFractionalBits);

    std::vector<dai::NeuralDepthConfig::PostProcessing::Filter> filtersToExecute;
    for(auto filter : postProcessing.filteringOrder) {
        switch(filter) {
            case NeuralDepthConfig::PostProcessing::Filter::SPECKLE:
                if(postProcessing.speckleFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case NeuralDepthConfig::PostProcessing::Filter::TEMPORAL:
                if(postProcessing.temporalFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case NeuralDepthConfig::PostProcessing::Filter::SPATIAL:
                if(postProcessing.spatialFilter.enable) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case NeuralDepthConfig::PostProcessing::Filter::DECIMATION:
                if(postProcessing.decimationFilter.decimationFactor > 1) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case NeuralDepthConfig::PostProcessing::Filter::MEDIAN:
                if(postProcessing.median != MedianFilter::MEDIAN_OFF) {
                    filtersToExecute.push_back(filter);
                }
                break;
            case NeuralDepthConfig::PostProcessing::Filter::NONE:
                break;
            default:
                break;
        }
    }

    if(filtersToExecute.size() != 0) {
        if(filtersToExecute.back() != NeuralDepthConfig::PostProcessing::Filter::MEDIAN) {
            maxDisp = maxDisp * ((1 << 13) / maxDisp);
        }
    }

    return maxDisp;
}
}  // namespace dai
