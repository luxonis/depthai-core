#include "depthai/pipeline/datatype/GPUStereoConfig.hpp"

namespace dai {

GPUStereoConfig::~GPUStereoConfig() = default;

void GPUStereoConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::GPUStereoConfig;
}

}  // namespace dai
