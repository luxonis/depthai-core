#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"

namespace dai {
DynamicCalibrationControl::~DynamicCalibrationControl() = default;  // Avoid weak vtable

void DynamicCalibrationControl::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}
}  // namespace dai
