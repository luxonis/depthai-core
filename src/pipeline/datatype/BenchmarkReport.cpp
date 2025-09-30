#include "depthai/pipeline/datatype/BenchmarkReport.hpp"

namespace dai {

BenchmarkReport::~BenchmarkReport() = default;

void BenchmarkReport::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::BenchmarkReport;
};
}  // namespace dai
