#include "CbaSensorDriver.hpp"

#include <array>
#include <cctype>
#include <utility>

namespace dai {
namespace gate {

namespace {

// Camera board code to camera HAL sensor driver name. The codes are the "C<nn>" fields of the board config in the
// "OAK EEPROM content" sheet (Board tab) and in depthai-boards-private batch/eeprom/SL0271_P0D1_oak_ffc_v2_*.json.
constexpr std::array<std::pair<std::string_view, std::string_view>, 8> CBA_SENSOR_DRIVERS{{
    {"C53", "og05b10"},  // OG05B20 colour, OAK-FFC-V2-OG05-C*
    {"C54", "og05b1b"},  // OG05B2B mono, OAK-FFC-V2-OG05-M*
    {"C57", "og05b10"},  // OG05B10
    {"C04", "ov9282"},   // OAK-FFC-OV9282*
    {"C14", "ov9782"},
    {"C34", "ar0234"},  // OAK-FFC-V2-AR0234-M12
    {"C43", "imx582"},  // OAK-FFC-48MP
    {"C45", "imx586"},  // OAK-FFC-V2-IMX586*
}};

bool isCameraCode(std::string_view field) {
    return field.size() >= 3 && field[0] == 'C' && std::isdigit(static_cast<unsigned char>(field[1])) && std::isdigit(static_cast<unsigned char>(field[2]));
}

}  // namespace

std::optional<std::string_view> cbaSensorDriverForBoardConf(std::string_view boardConf) {
    // The board config is '-' separated, the camera code starts one of the fields: "nIR-C54M00-10"
    while(!boardConf.empty()) {
        const auto end = boardConf.find('-');
        const auto field = boardConf.substr(0, end);
        if(isCameraCode(field)) {
            const auto code = field.substr(0, 3);
            for(const auto& [boardCode, driver] : CBA_SENSOR_DRIVERS) {
                if(boardCode == code) {
                    return driver;
                }
            }
            return std::nullopt;
        }
        boardConf = end == std::string_view::npos ? std::string_view{} : boardConf.substr(end + 1);
    }
    return std::nullopt;
}

}  // namespace gate
}  // namespace dai
