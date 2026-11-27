#pragma once

#include "depthai/common/EepromData.hpp"

namespace dai {
namespace utility {

std::vector<std::string> split(const std::string& s, char delimiter);

std::string parseProductName(const EepromData& eeprom, const EepromData& eepromFactory = {});  // eepromFactory takes precedence; empty if not found

std::string parseDeviceName(const EepromData& eeprom, const EepromData& eepromFactory = {});  // eepromFactory takes precedence; empty if not found

}  // namespace utility
}  // namespace dai
