#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

#include <stdexcept>

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

namespace {
struct DisparityEncoding {
    int maximum;
    int unitsPerPixel;
};

DisparityEncoding getDisparityEncoding(const StereoDepthConfig& config) {
    using Filter = StereoDepthConfig::PostProcessing::Filter;
    const auto bits = config.getSubpixel() ? config.getSubpixelFractionalBits() : 0;
    if(config.getSubpixel() && (bits < 3 || bits > 5)) {
        throw std::invalid_argument("RVC2 subpixel fractional bits must be 3, 4 or 5");
    }
    const int subpixel = 1 << bits;
    const int searchMaximum = config.costMatching.disparityWidth == StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64 ? 63 : 95;
    // Firmware normalizes the encoded matcher range. Companding and disparity
    // shift affect the depth mapping, not this encoded range.
    const int nominalMaximum = searchMaximum * (config.getExtendedDisparity() ? 2 : 1) * subpixel;
    const auto& post = config.postProcessing;
    const bool medianEnabled = post.median != StereoDepthConfig::MedianFilter::MEDIAN_OFF && nominalMaximum < 1024;
    auto lastEnabled = Filter::NONE;
    for(const auto filter : post.filteringOrder) {
        const bool enabled = (filter == Filter::SPECKLE && post.speckleFilter.enable) || (filter == Filter::SPATIAL && post.spatialFilter.enable)
                             || (filter == Filter::TEMPORAL && post.temporalFilter.enable)
                             || (filter == Filter::DECIMATION && post.decimationFilter.decimationFactor > 1) || (filter == Filter::MEDIAN && medianEnabled);
        if(enabled) lastEnabled = filter;
    }
    // Match the integer division in RVC2 Stereo::Init. A terminal median
    // coalesces with the depth LUT and restores nominal disparity units.
    const int normalization = lastEnabled != Filter::NONE && lastEnabled != Filter::MEDIAN ? (1 << 13) / nominalMaximum : 1;
    return {nominalMaximum * normalization, subpixel * normalization};
}
}  // namespace

float StereoDepthConfig::getMaxDisparity() const {
    return static_cast<float>(getDisparityEncoding(*this).maximum);
}

float StereoDepthConfig::getDisparityScale() const {
    if(costMatching.enableCompanding) {
        throw std::invalid_argument("Companded disparity has no single pixel scale");
    }
    return static_cast<float>(getDisparityEncoding(*this).unitsPerPixel);
}
}  // namespace dai
