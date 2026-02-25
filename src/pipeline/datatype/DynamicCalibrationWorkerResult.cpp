#include "depthai/pipeline/datatype/DynamicCalibrationWorkerResult.hpp"

namespace dai {

void DynamicCalibrationWorkerResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

DynamicCalibrationWorkerResult::~DynamicCalibrationWorkerResult() = default;

};  // namespace dai
