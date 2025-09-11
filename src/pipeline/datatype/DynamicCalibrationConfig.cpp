#include "depthai/pipeline/datatype/DynamicCalibrationConfig.hpp"

namespace dai {

DynamicCalibrationControl::~DynamicCalibrationControl() = default;

void DynamicCalibrationControl::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::DynamicCalibrationControl;
}

}  // namespace dai
