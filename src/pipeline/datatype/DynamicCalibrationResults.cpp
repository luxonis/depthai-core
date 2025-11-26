#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"

namespace dai {

CoverageData::~CoverageData() = default;

void CoverageData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

CalibrationQuality::~CalibrationQuality() = default;

void CalibrationQuality::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

DynamicCalibrationResult::~DynamicCalibrationResult() = default;

void DynamicCalibrationResult::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

}  // namespace dai