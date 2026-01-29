#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class SegmentationParserConfig : public Buffer {
   public:
    float confidenceThreshold = -1.0f;
    unsigned int stepSize = 1;

    /**
     * Construct SegmentationParserConfig message.
     */
    SegmentationParserConfig() = default;
    virtual ~SegmentationParserConfig();

    /**
     * Add a confidence threshold to the argmax operation over the segmentation tensor.
     * Pixels with confidence values below this threshold will be assigned the background class (255).
     * @param threshold Confidence threshold for segmentation parsing
     * @note Default is -1.0f, which means no thresholding is applied.
     * @note Only applicable if output classes are not in a single layer (eg. classesInOneLayer = false).
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Get confidence threshold
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the step size for segmentation parsing.
     * A step size of 1 means every pixel is processed, a step size of 2 means every second pixel is processed, and so on.
     * This can be used to speed up processing at the cost of lower resolution masks.
     * @param stepSize Step size for segmentation parsing
     */
    void setStepSize(unsigned int stepSize);

    /**
     * Gets the step size for segmentation parsing.
     */
    unsigned int getStepSize() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SegmentationParserConfig;
    }

    DEPTHAI_SERIALIZE(SegmentationParserConfig, confidenceThreshold, stepSize);
};

}  // namespace dai
