#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"

namespace dai {

void MultiDeviceCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

MultiDeviceCalibrationResult::~MultiDeviceCalibrationResult() = default;

CalibrationHandler MultiDeviceCalibrationResult::getCalibrationHandler() const {
    return CalibrationHandler(calibration);
}

}  // namespace dai
