#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"

namespace dai {

MultiDeviceCalibrationResult::~MultiDeviceCalibrationResult() = default;

void MultiDeviceCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

}  // namespace dai
