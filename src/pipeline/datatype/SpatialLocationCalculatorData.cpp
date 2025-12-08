#include "depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp"

namespace dai {

SpatialLocationCalculatorData::~SpatialLocationCalculatorData() = default;

void SpatialLocationCalculatorData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SpatialLocationCalculatorData;
}

std::vector<SpatialLocations> SpatialLocationCalculatorData::getSpatialLocations() const {
    return spatialLocations;
}
}  // namespace dai
