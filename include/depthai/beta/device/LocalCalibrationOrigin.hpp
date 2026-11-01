#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/EepromData.hpp"
#include "depthai/device/Platform.hpp"

namespace dai {
namespace beta {

/**
 * Describe a device's local calibration for
 * MultiDeviceCalibrationHandler::validateLocalOrigin.
 *
 * The origin socket follows the rule the platform's frame sources use when
 * they publish extrinsics, so the check mirrors what the frames will carry:
 *  - RVC2 ISP sources express every pose relative to the lowest-id socket.
 *  - RVC4 (and the host) express a pose relative to the extrinsics chain root,
 *    the socket reached by following toCameraSocket until AUTO.
 *
 * Kept header-only and free of CalibrationHandler so that both firmwares and
 * the host run this exact function.
 *
 * @param calibration The local calibration, or std::nullopt when unavailable
 * @param platform The platform whose frame sources publish the extrinsics
 */
inline LocalCalibrationOrigin describeLocalCalibration(const std::optional<EepromData>& calibration, Platform platform) {
    LocalCalibrationOrigin local;
    if(!calibration.has_value()) {
        return local;
    }
    const auto eeprom = std::make_shared<const EepromData>(*calibration);
    local.available = true;
    local.hasSocket = [eeprom](CameraBoardSocket socket) { return eeprom->cameraData.count(socket) > 0; };

    if(platform == Platform::RVC2) {
        local.originRule = "lowest-id socket";
        local.originOf = [eeprom](CameraBoardSocket) {
            if(eeprom->cameraData.empty()) {
                throw std::runtime_error("No camera data available in the calibration data.");
            }
            return std::min_element(eeprom->cameraData.begin(),
                                    eeprom->cameraData.end(),
                                    [](const auto& a, const auto& b) { return static_cast<int>(a.first) < static_cast<int>(b.first); })
                ->first;
        };
        return local;
    }

    local.originRule = "extrinsics chain root";
    local.originOf = [eeprom](CameraBoardSocket socket) {
        // Same traversal as CalibrationHandler::getExtrinsicsToOrigin: follow the chain to the
        // socket whose target is AUTO, rejecting dangling links and cycles.
        std::vector<CameraBoardSocket> path;
        CameraBoardSocket current = socket;
        while(true) {
            const auto it = eeprom->cameraData.find(current);
            if(it == eeprom->cameraData.end()) {
                throw std::runtime_error("Invalid camera link detected at camera " + toString(current) + " in the calibration data.");
            }
            path.push_back(current);
            const CameraBoardSocket next = it->second.extrinsics.toCameraSocket;
            if(next == CameraBoardSocket::AUTO) {
                return current;
            }
            if(std::find(path.begin(), path.end(), next) != path.end()) {
                throw std::runtime_error("Cyclic extrinsics detected in device calibration data.");
            }
            current = next;
        }
    };
    return local;
}

}  // namespace beta
}  // namespace dai
