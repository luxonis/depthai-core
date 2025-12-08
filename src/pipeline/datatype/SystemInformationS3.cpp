#include "depthai/pipeline/datatype/SystemInformationS3.hpp"

namespace dai {

SystemInformationS3::~SystemInformationS3() = default;

void SystemInformationS3::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::SystemInformationS3;
}

}  // namespace dai
