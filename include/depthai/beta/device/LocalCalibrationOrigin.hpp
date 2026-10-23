#pragma once

#include <memory>
#include <optional>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"

namespace dai {
namespace beta {

/**
 * Describe a device's local calibration for
 * MultiDeviceCalibrationHandler::validateLocalOrigin, using the extrinsics
 * chain root as the origin of every socket.
 *
 * This is the rule the host and the RVC4 frame sources follow when they
 * publish extrinsics. Kept header-only so firmware that does not compile
 * CalibrationHandler can still use the handler.
 *
 * @param calibration The local calibration, or std::nullopt when unavailable
 */
inline LocalCalibrationOrigin describeLocalCalibration(const std::optional<CalibrationHandler>& calibration) {
    LocalCalibrationOrigin local;
    if(!calibration.has_value()) {
        return local;
    }
    const auto handler = std::make_shared<const CalibrationHandler>(*calibration);
    local.available = true;
    local.hasSocket = [handler](CameraBoardSocket socket) { return handler->hasCameraCalibration(socket); };
    local.originOf = [handler](CameraBoardSocket socket) {
        // getExtrinsicsToOrigin always writes a concrete socket into origin or throws.
        CameraBoardSocket origin = CameraBoardSocket::AUTO;
        handler->getExtrinsicsToOrigin(socket, false, origin);
        return origin;
    };
    return local;
}

}  // namespace beta
}  // namespace dai
