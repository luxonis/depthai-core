#pragma once

#include <cstdint>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for ClassificationSequenceParser.
 */
class ClassificationSequenceParserConfig : public Buffer {
   public:
    std::vector<std::int32_t> ignoredIndexes;

    bool removeDuplicates = false;

    bool concatenateClasses = false;

    ClassificationSequenceParserConfig() = default;

    ~ClassificationSequenceParserConfig() override;

    /**
     * Sets the class indexes ignored while decoding the sequence.
     * @param indexes Nonnegative class indexes to ignore
     */
    void setIgnoredIndexes(const std::vector<std::int32_t>& indexes);

    /**
     * Gets the class indexes ignored while decoding the sequence.
     * @return Ignored class indexes
     */
    std::vector<std::int32_t> getIgnoredIndexes() const;

    /**
     * Sets whether consecutive duplicate classes are removed.
     * @param removeDuplicates Whether duplicate classes are removed
     */
    void setRemoveDuplicates(bool removeDuplicates);

    /**
     * Gets whether consecutive duplicate classes are removed.
     * @return Whether duplicate classes are removed
     */
    bool getRemoveDuplicates() const;

    /**
     * Sets whether decoded class labels are concatenated.
     * @param concatenateClasses Whether class labels are concatenated
     */
    void setConcatenateClasses(bool concatenateClasses);

    /**
     * Gets whether decoded class labels are concatenated.
     * @return Whether class labels are concatenated
     */
    bool getConcatenateClasses() const;

    /**
     * Validates this configuration.
     * @return True if all ignored indexes are nonnegative
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
        return DatatypeEnum::ClassificationSequenceParserConfig;
    }

    DEPTHAI_SERIALIZE(ClassificationSequenceParserConfig, ignoredIndexes, removeDuplicates, concatenateClasses);
};

}  // namespace beta
}  // namespace dai
