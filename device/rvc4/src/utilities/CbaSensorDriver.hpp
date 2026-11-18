#pragma once

#include <optional>
#include <string_view>

namespace dai {
namespace gate {

// Camera HAL sensor driver (its sensor name) of the camera board that a CBA EEPROM board config names, e.g. "nIR-C54M00-10"
// is a C54 board and uses "og05b1b". Empty for anything but the single-sensor camera boards in the table.
std::optional<std::string_view> cbaSensorDriverForBoardConf(std::string_view boardConf);

}  // namespace gate
}  // namespace dai
