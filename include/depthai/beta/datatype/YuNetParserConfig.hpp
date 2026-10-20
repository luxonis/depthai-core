#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for YuNetParser.
 */
class YuNetParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.8f;

    float iouThreshold = 0.3f;

    int maxDetections = 5000;

    YuNetParserConfig() = default;

    ~YuNetParserConfig() override;

    /**
     * Set the minimum face detection confidence.
     * @param threshold Confidence threshold in the inclusive range [0, 1]
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Get the minimum face detection confidence.
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
     * Set the maximum number of detections to retain.
     * @param maxDetections Maximum detection count; a value less than or equal to zero means unlimited
     */
    void setMaxDetections(int maxDetections);

    /**
     * Get the maximum number of detections to retain.
     * @return Maximum detection count; a value less than or equal to zero means unlimited
     */
    int getMaxDetections() const;

    /**
     * Check whether all configuration values are valid.
     * @return True when confidenceThreshold and iouThreshold are in the inclusive range [0, 1]; maxDetections may have any integer value
     */
    bool validate() const;

    /**
     * Serialize this configuration into stream metadata.
     * @param metadata Output buffer that receives the serialized configuration
     * @param datatype Output datatype identifier, set to YuNetParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for YuNetParserConfig.
     * @return DatatypeEnum::YuNetParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::YuNetParserConfig;
    }

    DEPTHAI_SERIALIZE(YuNetParserConfig, confidenceThreshold, iouThreshold, maxDetections);
};

}  // namespace beta
}  // namespace dai
