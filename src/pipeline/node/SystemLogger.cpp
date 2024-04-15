#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

void SystemLogger::build() {
    // Set some default properties
    properties.rateHz = 1.0f;
    // Link pool to inputPool
    // TODO(before mainline) - Add pools if ready
    // pool->out.link(inputPool);
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

float SystemLogger::getRate() {
    return properties.rateHz;
}

}  // namespace node
}  // namespace dai
