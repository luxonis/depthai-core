#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {

TrackedFeatures::~TrackedFeatures() = default;

void TrackedFeatures::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::TrackedFeatures;
}

}  // namespace dai
