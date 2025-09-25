#include "depthai/pipeline/datatype/AprilTags.hpp"

namespace dai {

AprilTags::~AprilTags() = default;

void AprilTags::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::AprilTags;
};

}  // namespace dai
