#include "StereoPairs.hpp"

namespace dai {
namespace gate {

std::vector<dai::StereoPair> getStereoPairs(dai::EepromData eepromData) {
    std::vector<dai::StereoPair> stereoPairs;
    if(eepromData.boardName.find("NG9498-ASM") != std::string::npos) {  // OAK4D
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    } else if(eepromData.boardName.find("PA9496") != std::string::npos) {  // OAK4-LITE
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    }
    return stereoPairs;
}

}  // namespace gate
}  // namespace dai
