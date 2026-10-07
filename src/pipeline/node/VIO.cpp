#include "depthai/pipeline/node/VIO.hpp"

#include <stdexcept>

namespace dai {
namespace node {

VIO& VIO::setImuUpdateRate(int rate) {
    if(rate <= 0) throw std::invalid_argument("VIO IMU frequency must be positive.");
    properties.imuFrequency = rate;
    return *this;
}

VIO& VIO::setUseSpecTranslation(bool use) {
    properties.useSpecTranslation = use;
    return *this;
}

void VIO::buildStage1() {
    if(!device || device->getPlatform() != Platform::RVC4) {
        throw std::runtime_error("VIO requires an RVC4 device with VIO-enabled firmware.");
    }
}

}  // namespace node
}  // namespace dai
