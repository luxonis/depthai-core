#include "depthai/device/CalibrationHandler.hpp"
#include "nlohmann/json.hpp"
#include <fstream>
#include <string>
namespace dai {

CalibrationHandler::CalibrationHandler(std::string eepromDataPath) {
    std::ifstream jsonStream(eepromDataPath);
    // TODO(sachin): Check if the file exists first.
    nlohmann::json json_data = nlohmann::json::parse(jsonStream);
    eepromData = json_data;
    // validateCameraArray();
}

CalibrationHandler::CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath) {
    std::ifstream jsonStream(boardConfigPath);
    nlohmann::json json_data = nlohmann::json::parse(jsonStream);
}

CalibrationHandler::CalibrationHandler(EepromData eepromData) {
    this->eepromData = eepromData;
}

dai::EepromData CalibrationHandler::getEepromData() const{
    return eepromData;
}
}  // namespace dai