#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for SuperAnimalParser.
 */
class SuperAnimalParserConfig : public Buffer {
   public:
    float scoreThreshold = 0.5f;

    SuperAnimalParserConfig() = default;

    ~SuperAnimalParserConfig() override;

    /**
     * Set the minimum keypoint score.
     * @param threshold Score threshold in the inclusive range [0, 1]
     */
    void setScoreThreshold(float threshold);

    /**
     * Get the minimum keypoint score.
     * @return Score threshold in the inclusive range [0, 1]
     */
    float getScoreThreshold() const;

    /**
     * Check whether all configuration values are valid.
     * @return True when scoreThreshold is in the inclusive range [0, 1]
     */
    bool validate() const;

    /**
     * Serialize this configuration into stream metadata.
     * @param metadata Output buffer that receives the serialized configuration
     * @param datatype Output datatype identifier, set to SuperAnimalParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for SuperAnimalParserConfig.
     * @return DatatypeEnum::SuperAnimalParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::SuperAnimalParserConfig;
    }

    DEPTHAI_SERIALIZE(SuperAnimalParserConfig, scoreThreshold);
};

}  // namespace beta
}  // namespace dai
