#include "depthai/beta/datatype/MultiDeviceCalibrationControl.hpp"

namespace dai {
namespace beta {

MultiDeviceCalibrationControl::~MultiDeviceCalibrationControl() = default;

void MultiDeviceCalibrationControl::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

}  // namespace beta
}  // namespace dai
