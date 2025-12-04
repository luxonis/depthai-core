#include "depthai/pipeline/datatype/ThermalConfig.hpp"

namespace dai {

ThermalConfig::~ThermalConfig() = default;

void ThermalConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::ThermalConfig;
}

}  // namespace dai