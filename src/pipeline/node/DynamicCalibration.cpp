#include "depthai/pipeline/node/DynamicCalibration.hpp"
#include "common/CameraBoardSocket.hpp"

namespace dai {
namespace node {


void DynamicCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool DynamicCalibration::runOnHost() const {
    return runOnHostVar;
}

/**
 * Get the calibration quality (epipolar error)
 * @return Epipolar error in pixels
 */
float DynamicCalibration::getCalibrationQuality() const {
    // Placeholder for actual calibration quality calculation
    return 0.0f;
}

void DynamicCalibration::run() {
    if(!device) {
        std::cout << "Dynamic calibration node has to have access to a device!" << std::endl;
        return;
    }
    auto currentCalibration = device->readCalibration();
    while(isRunning()) {
        auto leftFrame = left.get<dai::ImgFrame>();
        auto rightFrame = right.get<dai::ImgFrame>();
        auto extrinsicsLeftToRight = currentCalibration.getCameraExtrinsics(static_cast<dai::CameraBoardSocket>(leftFrame->instanceNum),
                                                                             static_cast<dai::CameraBoardSocket>(rightFrame->instanceNum));
        // Make sure the fra
        std::cout << "DynamicCalibration node is running" << std::endl;
    }
}
}  // namespace node
}  // namespace dai
