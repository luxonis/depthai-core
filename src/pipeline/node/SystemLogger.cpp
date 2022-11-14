#include "depthai/pipeline/node/SystemLogger.hpp"

namespace dai {
namespace node {

SystemLogger::SystemLogger() : NodeCRTP<DeviceNode, SystemLogger, SystemLoggerProperties>() {
    properties.rateHz = 1.0f;
    setOutputRefs(&out);
}

SystemLogger::SystemLogger(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, SystemLogger, SystemLoggerProperties>(std::move(props)) {
    setOutputRefs(&out);
}

void SystemLogger::setRate(float hz) {
    properties.rateHz = hz;
}

float SystemLogger::getRate() {
    return properties.rateHz;
}

}  // namespace node
}  // namespace dai
