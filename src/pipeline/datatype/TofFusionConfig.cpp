#include "depthai/pipeline/datatype/TofFusionConfig.hpp"

namespace dai {

TofFusionConfig::~TofFusionConfig() = default;

void TofFusionConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::TofFusionConfig;
}

}  // namespace dai
