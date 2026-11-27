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

    if(multiDeviceCalibration == nullptr || extrinsics.toCameraSocket == CameraBoardSocket::AUTO || extrinsics.rotationMatrix.empty()) {
        return transformation;
    }

    try {
        const auto bridge = multiDeviceCalibration->getExtrinsicsToOrigin(deviceId, extrinsics.toCameraSocket);
        if(bridge.has_value()) {
            transformation.rebaseExtrinsics(*bridge);
        }
    } catch(const std::exception& ex) {
        if(!warnedForMismatch && logger) {  // Should not happen (the RPC handlers reject mismatches): keep local metadata, do not drop frames
            warnedForMismatch = true;
            logger->warn("Multi-device calibration not applied for socket {}: {}", toString(extrinsics.toCameraSocket), ex.what());
        }
    }
    return transformation;
}

}  // namespace gate
}  // namespace dai
