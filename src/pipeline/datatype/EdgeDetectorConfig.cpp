#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

namespace dai {

EdgeDetectorConfig::~EdgeDetectorConfig() = default;

void EdgeDetectorConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = this->getDatatype();
}

void EdgeDetectorConfig::setSobelFilterKernels(const std::vector<std::vector<int>>& horizontalKernel, const std::vector<std::vector<int>>& verticalKernel) {
    config.sobelFilterHorizontalKernel = horizontalKernel;
    config.sobelFilterVerticalKernel = verticalKernel;
}

EdgeDetectorConfig::EdgeDetectorConfigData EdgeDetectorConfig::getConfigData() const {
    return config;
}
}  // namespace dai
