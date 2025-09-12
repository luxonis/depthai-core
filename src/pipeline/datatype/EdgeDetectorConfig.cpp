#include "depthai/pipeline/datatype/EdgeDetectorConfig.hpp"

namespace dai {

#if defined(__clang__)
EdgeDetectorConfig::~EdgeDetectorConfig() = default;
#endif

void EdgeDetectorConfig::setSobelFilterKernels(const std::vector<std::vector<int>>& horizontalKernel, const std::vector<std::vector<int>>& verticalKernel) {
    config.sobelFilterHorizontalKernel = horizontalKernel;
    config.sobelFilterVerticalKernel = verticalKernel;
}

EdgeDetectorConfig::EdgeDetectorConfigData EdgeDetectorConfig::getConfigData() const {
    return config;
}
}  // namespace dai
