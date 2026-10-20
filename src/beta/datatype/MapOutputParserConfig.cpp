#include "depthai/beta/datatype/MapOutputParserConfig.hpp"

namespace dai {
namespace beta {

MapOutputParserConfig::~MapOutputParserConfig() = default;

void MapOutputParserConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

bool MapOutputParserConfig::validate() const {
    return true;
}

void MapOutputParserConfig::setMinMaxScaling(bool enabled) {
    minMaxScaling = enabled;
}

bool MapOutputParserConfig::getMinMaxScaling() const {
    return minMaxScaling;
}

}  // namespace beta
}  // namespace dai
