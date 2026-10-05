#include "depthai/beta/datatype/MultiDeviceCalibrationResult.hpp"

namespace dai {
namespace beta {

MultiDeviceCalibrationResult::~MultiDeviceCalibrationResult() = default;

std::optional<MultiDeviceCalibrationHandler> MultiDeviceCalibrationResult::getHandler() const {
    if(!graph.has_value()) {
        return std::nullopt;
    }
    return MultiDeviceCalibrationHandler(*graph);
}

void MultiDeviceCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = getDatatype();
}

}  // namespace beta
}  // namespace dai
