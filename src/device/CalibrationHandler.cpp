#include "depthai/device/CalibrationHandler.hpp"



namespace dai {

    CalibrationHandler::CalibrationHandler(std::string eepromDataPath){
        std::ifstream jsonStream(eepromDataPath);
        json json_data = json::parse(ifs);
        eepromData = json_data;
        // validateCameraArray();
    }

    CalibrationHandler::CalibrationHandler(std::string calibrationDataPath, std::string boardConfigPath){
        std::ifstream jsonStream(boardConfigPath);
        json json_data = json::parse(ifs);
    }

    CalibrationHandler::CalibrationHandler(EepromData eepromData){
        this->eepromData = eepromData;
    }
}