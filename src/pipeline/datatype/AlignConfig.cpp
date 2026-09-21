#include "depthai/pipeline/datatype/AlignConfig.hpp"

namespace dai {

AlignConfig::~AlignConfig() = default;

void AlignConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::AlignConfig;
}

}  // namespace dai
