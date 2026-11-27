#include "ImgTransformationUtils.hpp"

#include <stdexcept>

namespace dai {
namespace gate {

ImgTransformation prepareMultiDeviceTransformation(ImgTransformation transformation,
                                                   const std::string& deviceId,
                                                   const beta::MultiDeviceCalibrationHandler* multiDeviceCalibration,
                                                   bool& warnedForMismatch,
                                                   const std::shared_ptr<spdlog::logger>& logger) {
    if(deviceId.empty() || deviceId == "xlink-server") {
        return transformation;
    }

    auto extrinsics = transformation.getExtrinsics();
    extrinsics.toDeviceId = deviceId;
    transformation.setExtrinsics(extrinsics);

    // An intrinsics-only transformation has no local 3D pose to rebase.
    if(multiDeviceCalibration == nullptr || extrinsics.toCameraSocket == CameraBoardSocket::AUTO || extrinsics.rotationMatrix.empty()) {
        return transformation;
    }

    try {
        const auto bridge = multiDeviceCalibration->getExtrinsicsToOrigin(deviceId, extrinsics.toCameraSocket);
        if(bridge.has_value()) {
            transformation.rebaseExtrinsics(*bridge);
        }
    } catch(const std::exception& ex) {
        // Safety net: the RPC handlers reject mismatching graphs and calibrations, so this
        // should not happen. Keep publishing local metadata rather than dropping frames.
        if(!warnedForMismatch && logger) {
            warnedForMismatch = true;
            logger->warn("Multi-device calibration not applied for socket {}: {}", toString(extrinsics.toCameraSocket), ex.what());
        }
    }
    return transformation;
}

}  // namespace gate
}  // namespace dai
