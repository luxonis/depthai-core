#pragma once

#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

struct FocusedDepthCropProperties : PropertiesSerializable<Properties, FocusedDepthCropProperties> {
    /// Disparity search margin in pixels (typically StereoDepthConfig::getMaxDisparity()).
    float maxDisparityPixels = 96.F;
    uint32_t minCropWidth = 16;
    uint32_t minCropHeight = 16;
    /// When non-zero, apply letterbox resize to this size after the symmetric crop (NeuralDepth path).
    uint32_t neuralResizeWidth = 0;
    uint32_t neuralResizeHeight = 0;
    ImageManipProperties imageManip;

    ~FocusedDepthCropProperties() override;
};

DEPTHAI_SERIALIZE_EXT(FocusedDepthCropProperties, maxDisparityPixels, minCropWidth, minCropHeight, neuralResizeWidth, neuralResizeHeight, imageManip);

}  // namespace dai
