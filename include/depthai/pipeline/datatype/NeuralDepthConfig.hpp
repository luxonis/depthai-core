#pragma once

#include <depthai/common/ProcessorType.hpp>
#include <depthai/common/optional.hpp>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * NeuralDepthConfig message.
 */
class NeuralDepthConfig : public Buffer {
   public:
    /**
     * Construct NeuralDepthConfig message.
     */
    NeuralDepthConfig() = default;
    virtual ~NeuralDepthConfig();

    struct AlgorithmControl {
        /**
         * Align the disparity/depth to the perspective of a rectified output, or center it
         */
        enum class DepthAlign : int32_t { RECTIFIED_RIGHT, RECTIFIED_LEFT, CENTER };

        /**
         * Measurement unit for depth data
         */
        enum class DepthUnit : int32_t { METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM };

        /**
         * Measurement unit for depth data.
         * Depth data is integer value, multiple of depth unit.
         */
        DepthUnit depthUnit = DepthUnit::MILLIMETER;

        /**
         * Custom depth unit multiplier, if custom depth unit is enabled, relative to 1 meter.
         * A multiplier of 1000 effectively means depth unit in millimeter.
         */
        float customDepthUnitMultiplier = 1000.f;
        DEPTHAI_SERIALIZE(AlgorithmControl, depthUnit, customDepthUnitMultiplier);
    };

    struct Filtering {
        /**
         * Confidence threshold for disparity calculation,
         * Confidences above this value will be considered valid.
         * Valid range is [0,255].
         */
        uint8_t confidenceThreshold = 125;

        /**
         * Edge threshold for disparity calculation,
         * Pixels with edge magnitude below this value will be considered invalid.
         * Valid range is [0,255].
         */
        uint8_t edgeThreshold = 10;

        DEPTHAI_SERIALIZE(Filtering, confidenceThreshold, edgeThreshold);
    };

    /**
     * Confidence threshold for disparity calculation
     * @param confThr Confidence threshold value 0..255
     */
    NeuralDepthConfig& setConfidenceThreshold(uint8_t confThr);

    /**
     * Get confidence threshold for disparity calculation
     */
    uint8_t getConfidenceThreshold() const;

    /**
     * Set edge threshold for disparity calculation
     * @param edgeThr Edge threshold value 0..255
     */
    NeuralDepthConfig& setEdgeThreshold(uint8_t edgeThr);

    /**
     * Get edge threshold for disparity calculation
     */
    uint8_t getEdgeThreshold() const;

    /**
     * Controls the flow of stereo algorithm - left-right check, subpixel etc.
     */
    AlgorithmControl algorithmControl;

    /**
     * Controls the postprocessing of disparity and/or depth map.
     */
    Filtering postProcessing;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DEPTHAI_SERIALIZE(NeuralDepthConfig, algorithmControl, postProcessing);
};

}  // namespace dai
