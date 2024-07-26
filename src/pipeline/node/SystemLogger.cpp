#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

void SystemLogger::buildInternal() {
    properties.rateHz = 1.0f;
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

float SystemLogger::getRate() {
    return properties.rateHz;
}

}  // namespace node
}  // namespace dai
