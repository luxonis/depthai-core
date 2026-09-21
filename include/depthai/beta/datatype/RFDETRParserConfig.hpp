#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for RFDETRParser.
 */
class RFDETRParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.5f;

    int maxDetections = 300;

    float maskConfidence = 0.5f;

    RFDETRParserConfig() = default;

    ~RFDETRParserConfig() override;

    /**
     * Set the minimum detection confidence.
     * @param threshold Confidence threshold in the inclusive range [0, 1]
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Get the minimum detection confidence.
     * @return Confidence threshold in the inclusive range [0, 1]
     */
    float getConfidenceThreshold() const;

    /**
     * Set the maximum number of detections to retain.
     * @param maxDetections Maximum detection count, which must be positive
     */
    void setMaxDetections(int maxDetections);

    /**
     * Get the maximum number of detections to retain.
     * @return Maximum detection count
     */
    int getMaxDetections() const;

    /**
     * Set the minimum per-pixel confidence used when creating instance masks.
     * @param threshold Mask confidence threshold in the inclusive range [0, 1]
     */
    void setMaskConfidence(float threshold);

    /**
     * Get the minimum per-pixel confidence used when creating instance masks.
     * @return Mask confidence threshold in the inclusive range [0, 1]
     */
    float getMaskConfidence() const;

    /**
     * Check whether all configuration values are valid.
     * @return True when both confidence thresholds are in the inclusive range [0, 1] and maxDetections is positive
     */
    bool validate() const;

    /**
     * Serialize this configuration into stream metadata.
     * @param metadata Output buffer that receives the serialized configuration
     * @param datatype Output datatype identifier, set to RFDETRParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for RFDETRParserConfig.
     * @return DatatypeEnum::RFDETRParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::RFDETRParserConfig;
    }

    DEPTHAI_SERIALIZE(RFDETRParserConfig, confidenceThreshold, maxDetections, maskConfidence);
};

}  // namespace beta
}  // namespace dai
