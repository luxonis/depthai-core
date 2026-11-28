#include "DynamicCalibration.hpp"

#include <stdexcept>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "device/StereoPairUtils.hpp"
#include "utilities/CalibrationAccess.hpp"
#include "utilities/CameraDiscovery.hpp"
#include "utility/EepromDataParser.hpp"

namespace dai {
namespace gate {

CalibrationHandler DynamicCalibration::readFactoryCalibration() {
    auto [success, errorMessage, eepromData] = ::readFromEepromFactory(CameraBoardSocket::AUTO);
    if(!success) {
        throw std::runtime_error(errorMessage);
    }
    return CalibrationHandler(eepromData);
}

std::vector<StereoPair> DynamicCalibration::getStereoPairs() {
    return detail::findStereoPairs([this]() { return getParentPipeline().getCalibrationData(); },
                                   [this]() { return readFactoryCalibration(); },
                                   []() { return CameraDiscovery::getInstance().getConnectedCameraFeatures(); });
}

std::pair<std::string, std::string> DynamicCalibration::getProductAndBoardName() {
    EepromData eeprom;  // An unreadable area counts as empty, like dai::Device::getProductName() on the host
    EepromData eepromFactory;
    if(auto [success, errorMessage, data] = ::readFromEeprom(CameraBoardSocket::AUTO); success) {
        eeprom = data;
    }
    if(auto [success, errorMessage, data] = ::readFromEepromFactory(CameraBoardSocket::AUTO); success) {
        eepromFactory = data;
    }
    return {utility::parseProductName(eeprom, eepromFactory), eeprom.boardName};
}

void DynamicCalibration::applyCalibration(const CalibrationHandler& handler, bool flash) {
    auto eepromData = handler.getEepromData();
    auto [success, errorMessage] = ::setCalibration(eepromData);
    if(!success) {
        throw std::runtime_error("DynamicCalibration: failed to apply the calibration on the device: " + errorMessage);
    }
    if(flash) {  // User area only, like dai::Device::flashCalibration()
        auto [flashed, flashError] = ::storeToEeprom(eepromData, false, false, CameraBoardSocket::AUTO);
        if(!flashed) {
            throw std::runtime_error("DynamicCalibration: failed to flash the calibration: " + flashError);
        }
    }
}

}  // namespace gate
}  // namespace dai
