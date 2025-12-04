#include "depthai/pipeline/datatype/SystemInformation.hpp"

namespace dai {

SystemInformation::~SystemInformation() = default;

void SystemInformation::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SystemInformation;
}

}  // namespace dai
