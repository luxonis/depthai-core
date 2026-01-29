#include "depthai/pipeline/datatype/MapData.hpp"

#include "pipeline/datatype/ImgFrame.hpp"

namespace dai {
MapData::~MapData() = default;
void MapData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::MapData;
};
std::shared_ptr<dai::ImgFrame> MapData::getMap() {
    return std::dynamic_pointer_cast<dai::ImgFrame>(map);
}
void MapData::setMap(std::shared_ptr<ImgFrame> map) {
    this->map = map;
}
// No implementation needed
}  // namespace dai
