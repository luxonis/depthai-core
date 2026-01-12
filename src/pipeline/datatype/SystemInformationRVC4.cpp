#include "depthai/pipeline/datatype/SystemInformationRVC4.hpp"

namespace dai {

SystemInformationRVC4::~SystemInformationRVC4() = default;

void SystemInformationRVC4::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SystemInformationRVC4;
}

}  // namespace dai
