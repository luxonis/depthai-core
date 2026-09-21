#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for HRNetParser.
 */
class HRNetParserConfig : public Buffer {
   public:
    float scoreThreshold = 0.5f;

    HRNetParserConfig() = default;

    ~HRNetParserConfig() override;

    /**
     * Sets the minimum keypoint score.
     * @param threshold Score threshold in the range [0, 1]
     */
    void setScoreThreshold(float threshold);

    /**
     * Gets the minimum keypoint score.
     * @return Score threshold
     */
    float getScoreThreshold() const;

    /**
     * Validates this configuration.
     * @return True if the score threshold is in the range [0, 1]
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
        return DatatypeEnum::HRNetParserConfig;
    }

    DEPTHAI_SERIALIZE(HRNetParserConfig, scoreThreshold);
};

}  // namespace beta
}  // namespace dai
