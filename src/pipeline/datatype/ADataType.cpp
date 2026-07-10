#include "depthai/pipeline/datatype/ADatatype.hpp"

namespace dai {

ADatatypeInterface::~ADatatypeInterface() = default;
ADatatype::~ADatatype() = default;

void ADatatype::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    (void)metadata;
    datatype = this->getDatatype();
};

}  // namespace dai
