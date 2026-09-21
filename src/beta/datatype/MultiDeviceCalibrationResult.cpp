#include "depthai/beta/datatype/MultiDeviceCalibrationResult.hpp"

namespace dai {
namespace beta {

MultiDeviceCalibrationResult::~MultiDeviceCalibrationResult() = default;

void MultiDeviceCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

}  // namespace beta
}  // namespace dai
