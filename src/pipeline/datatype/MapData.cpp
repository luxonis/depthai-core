#include "depthai/pipeline/datatype/MapData.hpp"

namespace dai {
MapData::~MapData() = default;
void MapData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::MapData;
};
// No implementation needed
}  // namespace dai
