#include "depthai/pipeline/datatype/GateControl.hpp"

namespace dai {
GateControl::~GateControl() = default;  // Avoid weak vtable

void GateControl::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}
}  // namespace dai
