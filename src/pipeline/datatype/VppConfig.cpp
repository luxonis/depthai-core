#include "depthai/pipeline/datatype/VppConfig.hpp"

namespace dai {
VppConfig::~VppConfig() = default;

void VppConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::VppConfig;
}
}  // namespace dai
