#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

std::shared_ptr<SystemLogger> SystemLogger::build() {
    // Set some default properties
    properties.rateHz = 1.0f;
    // Link pool to inputPool
    // TODO(before mainline) - Add pools if ready
    // pool->out.link(inputPool);
    //
    isBuild = true;
    return std::static_pointer_cast<SystemLogger>(shared_from_this());
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

float SystemLogger::getRate() {
    return properties.rateHz;
}

}  // namespace node
}  // namespace dai
