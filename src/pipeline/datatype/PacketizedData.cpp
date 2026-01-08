#include "PacketizedData.hpp"

namespace dai {
PacketizedData::~PacketizedData() = default;

void PacketizedData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PacketizedData;
}
}  // namespace dai
