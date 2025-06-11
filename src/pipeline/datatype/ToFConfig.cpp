#include "depthai/pipeline/datatype/ToFConfig.hpp"

namespace dai {

ToFConfig& ToFConfig::setMedianFilter(MedianFilter median) {
    this->median = median;
    return *this;
}

}  // namespace dai
