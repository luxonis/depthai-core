#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for MLSDParser.
 */
class MLSDParserConfig : public Buffer {
   public:
    int topK = 200;

    float scoreThreshold = 0.10f;

    float distanceThreshold = 20.0f;

    MLSDParserConfig() = default;

    ~MLSDParserConfig() override;

    /**
     * Sets the number of highest-scoring candidates retained for decoding.
     * @param topK Positive candidate count
     */
    void setTopK(int topK);

    /**
     * Gets the number of highest-scoring candidates retained for decoding.
     * @return Candidate count
     */
    int getTopK() const;

    /**
     * Sets the minimum candidate score.
     * @param threshold Score threshold in the range [0, 1]
     */
    void setScoreThreshold(float threshold);

    /**
     * Gets the minimum candidate score.
     * @return Score threshold
     */
    float getScoreThreshold() const;

    /**
     * Sets the distance threshold used while decoding line segments.
     * @param threshold Nonnegative distance threshold
     */
    void setDistanceThreshold(float threshold);

    /**
     * Gets the distance threshold used while decoding line segments.
     * @return Distance threshold
     */
    float getDistanceThreshold() const;

    /**
     * Validates this configuration.
     * @return True if topK is positive, the score threshold is in [0, 1], and the distance threshold is nonnegative
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
        return DatatypeEnum::MLSDParserConfig;
    }

    DEPTHAI_SERIALIZE(MLSDParserConfig, topK, scoreThreshold, distanceThreshold);
};

}  // namespace beta
}  // namespace dai
