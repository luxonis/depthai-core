#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for MPPalmDetectionParser.
 */
class MPPalmDetectionParserConfig : public Buffer {
   public:
    float confidenceThreshold = 0.5f;

    float iouThreshold = 0.5f;

    int maxDetections = 100;

    MPPalmDetectionParserConfig() = default;

    ~MPPalmDetectionParserConfig() override;

    /**
     * Sets the minimum detection confidence.
     * @param threshold Confidence threshold in the range [0, 1]
     */
    void setConfidenceThreshold(float threshold);

    /**
     * Gets the minimum detection confidence.
     * @return Confidence threshold
     */
    float getConfidenceThreshold() const;

    /**
     * Sets the intersection-over-union threshold used by non-maximum suppression.
     * @param threshold IoU threshold in the range [0, 1]
     */
    void setIouThreshold(float threshold);

    /**
     * Gets the intersection-over-union threshold.
     * @return IoU threshold
     */
    float getIouThreshold() const;

    /**
     * Sets the maximum number of emitted detections.
     * @param maxDetections Positive maximum detection count
     */
    void setMaxDetections(int maxDetections);

    /**
     * Gets the maximum number of emitted detections.
     * @return Maximum detection count
     */
    int getMaxDetections() const;

    /**
     * Validates this configuration.
     * @return True if both thresholds are in [0, 1] and maxDetections is positive
     */
    bool validate() const;

    /**
     * Serializes this configuration into message metadata.
     * @param metadata Destination metadata buffer
     * @param datatype Datatype identifier written during serialization
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Returns the datatype identifier for this configuration.
     * @return Configuration datatype identifier
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MPPalmDetectionParserConfig;
    }

    DEPTHAI_SERIALIZE(MPPalmDetectionParserConfig, confidenceThreshold, iouThreshold, maxDetections);
};

}  // namespace beta
}  // namespace dai
