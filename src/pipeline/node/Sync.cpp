#include "depthai/pipeline/node/Sync.hpp"

namespace dai {
namespace node {

void Sync::setSyncThreshold(std::chrono::nanoseconds syncThreshold) {
    properties.syncThresholdNs = syncThreshold.count();
}

void Sync::setSyncAttempts(int syncAttempts) {
    properties.syncAttempts = syncAttempts;
}

std::chrono::nanoseconds Sync::getSyncThreshold() const {
    return std::chrono::nanoseconds(properties.syncThresholdNs);
}

int Sync::getSyncAttempts() const {
    return properties.syncAttempts;
}

}  // namespace node
}  // namespace dai
