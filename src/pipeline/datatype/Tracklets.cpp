#include "depthai/pipeline/datatype/Tracklets.hpp"

namespace dai {

Tracklets::~Tracklets() = default;

void Tracklets::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::Tracklets;
}

}  // namespace dai
