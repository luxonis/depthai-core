#include <fmt/format.h>

#include <stdexcept>

#include "depthai/device/CalibrationHandler.hpp"

namespace dai {

namespace {

CoordinateFrame frameOf(const std::optional<std::string>& device, CameraBoardSocket socket) {
    return {device.value_or(""), socket};
}

}  // namespace

Extrinsics CalibrationHandler::getExtrinsics(const std::optional<std::string>& fromDevice,
                                             CameraBoardSocket fromSocket,
                                             const std::optional<std::string>& toDevice,
                                             CameraBoardSocket toSocket,
                                             LengthUnit unit) const {
    const auto from = frameOf(fromDevice, fromSocket);
    const auto to = frameOf(toDevice, toSocket);

    // Legacy EEPROM calibration is inherently single-device: an omitted device, or equal device IDs, selects it.
    if(from.deviceId == to.deviceId && hasCameraCalibration(from.socket) && hasCameraCalibration(to.socket)) {
        Extrinsics result(getCameraExtrinsics(from.socket, to.socket, false, unit), to.socket, unit);
        result.setReferenceFrame(to);
        return result;
    }

    const auto device = eepromData.devicesData.find(from.deviceId);
    if(device == eepromData.devicesData.end()) {
        throw std::runtime_error(fmt::format("No extrinsics stored from {} to {}.", toString(from), toString(to)));
    }
    const auto it = device->second.find(from.socket);
    if(it == device->second.end() || it->second.getReferenceFrame() != to) {
        throw std::runtime_error(fmt::format("No extrinsics stored from {} to {}.", toString(from), toString(to)));
    }
    Extrinsics result = it->second;
    result.setReferenceFrame(to);
    result.setTransformationMatrix(result.getTransformationMatrix(false, unit), unit);
    return result;
}

void CalibrationHandler::setExtrinsics(const std::optional<std::string>& fromDevice,
                                       CameraBoardSocket fromSocket,
                                       const std::optional<std::string>& toDevice,
                                       CameraBoardSocket toSocket,
                                       const Extrinsics& extrinsics,
                                       LengthUnit unit) {
    const auto from = frameOf(fromDevice, fromSocket);
    const auto to = frameOf(toDevice, toSocket);
    if(from == to) {
        throw std::invalid_argument(fmt::format("Cannot store extrinsics from {} to itself.", toString(from)));
    }

    Extrinsics stored = extrinsics;
    stored.setTransformationMatrix(extrinsics.getTransformationMatrix(false, unit), unit);
    stored.setReferenceFrame(to);
    eepromData.devicesData[from.deviceId][from.socket] = std::move(stored);
}

}  // namespace dai
