#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for MapOutputParser.
 */
class MapOutputParserConfig : public Buffer {
   public:
    bool minMaxScaling = false;

    MapOutputParserConfig() = default;

    ~MapOutputParserConfig() override;

    /**
     * Sets whether output values are scaled using their minimum and maximum.
     * @param enabled Whether min-max scaling is enabled
     */
    void setMinMaxScaling(bool enabled);

    /**
     * Gets whether output values are scaled using their minimum and maximum.
     * @return Whether min-max scaling is enabled
     */
    bool getMinMaxScaling() const;

    /**
     * Validates this configuration.
     * @return True because every value of the boolean option is valid
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
        return DatatypeEnum::MapOutputParserConfig;
    }

    DEPTHAI_SERIALIZE(MapOutputParserConfig, minMaxScaling);
};

}  // namespace beta
}  // namespace dai
