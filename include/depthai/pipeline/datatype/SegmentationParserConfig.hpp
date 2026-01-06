#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class SegmentationParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.5f;
    std::vector<std::string> labels = {};
    int stepSize = 1;

    /**
     * Construct SegmentationParserConfig message.
     */
    SegmentationParserConfig() = default;
    virtual ~SegmentationParserConfig();

    /**
     * Set confidence threshold for segmentation parsing
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Get confidence threshold for segmentation parsing
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the class labels associated with the segmentation mask.
     * The label at index $i$ in the `labels` vector corresponds to the value $i$ in the segmentation mask data array.
     * @param labels Vector of class labels
     */
    void setLabels(const std::vector<std::string>& labels);

    /**
     * Returns the class labels associated with the segmentation mask.
     */
    std::vector<std::string> getLabels() const;

    /**
     * Sets the step size for segmentation parsing.
     * A step size of 1 means every pixel is processed, a step size of 2 means every second pixel is processed, and so on.
     * This can be used to speed up processing at the cost of lower resolution masks.
     * @param stepSize Step size for segmentation parsing
     */
    void setStepSize(int stepSize);

    /**
     * Gets the step size for segmentation parsing.
     */
    int getStepSize() const;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SegmentationParserConfig;
    }

    DEPTHAI_SERIALIZE(SegmentationParserConfig, confidenceThreshold, labels, stepSize);
};

}  // namespace dai
