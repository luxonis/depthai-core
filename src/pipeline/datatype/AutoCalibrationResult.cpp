#include "depthai/pipeline/datatype/AutoCalibrationResult.hpp"

namespace dai {

void AutoCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

AutoCalibrationResult::~AutoCalibrationResult() = default;

};  // namespace dai
