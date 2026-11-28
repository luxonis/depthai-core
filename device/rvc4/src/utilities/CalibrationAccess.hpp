#pragma once

#include <functional>
#include <optional>
#include <string>
#include <tuple>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/EepromData.hpp"

extern std::function<std::tuple<bool, std::string>(std::optional<dai::EepromData>)> setCalibration;
extern std::function<std::tuple<bool, std::string>(dai::EepromData, bool, bool, dai::CameraBoardSocket)> storeToEeprom;
extern std::function<std::tuple<bool, std::string, dai::EepromData>(dai::CameraBoardSocket)> readFromEeprom;
extern std::function<std::tuple<bool, std::string, dai::EepromData>(dai::CameraBoardSocket)> readFromEepromFactory;
