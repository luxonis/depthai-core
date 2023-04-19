#include "depthai/pipeline/node/Sync.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

void Sync::setSyncThresholdMs(float thresholdMs) {
    properties.syncThresholdMs = thresholdMs;
}

}  // namespace node
}  // namespace dai
