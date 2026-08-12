#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for SCRFDParser.
 */
class SCRFDParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.5f;

    float iouThreshold = 0.5f;

    int maxDetections = 100;

    SCRFDParserConfig() = default;

    ~SCRFDParserConfig() override;

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
     * Set the non-maximum suppression intersection-over-union threshold.
     * @param threshold Intersection-over-union threshold in the inclusive range [0, 1]
     */
    void setIouThreshold(float threshold);

    /**
     * Get the non-maximum suppression intersection-over-union threshold.
     * @return Intersection-over-union threshold in the inclusive range [0, 1]
     */
    float getIouThreshold() const;

    /**
     * Set the maximum number of post-suppression detections to retain.
     * @param maxDetections Maximum detection count, which must be positive
     */
    void setMaxDetections(int maxDetections);

    /**
     * Get the maximum number of post-suppression detections to retain.
     * @return Maximum post-suppression detection count
     */
    int getMaxDetections() const;

    /**
     * Check whether all configuration values are valid.
     * @return True when both thresholds are in the inclusive range [0, 1] and maxDetections is positive
     */
    bool validate() const;

    /**
     * Serialize this configuration into stream metadata.
     * @param metadata Output buffer that receives the serialized configuration
     * @param datatype Output datatype identifier, set to SCRFDParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for SCRFDParserConfig.
     * @return DatatypeEnum::SCRFDParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SCRFDParserConfig;
    }

    DEPTHAI_SERIALIZE(SCRFDParserConfig, confidenceThreshold, iouThreshold, maxDetections);
};

}  // namespace beta
}  // namespace dai
