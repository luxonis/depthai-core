#include "StereoPairs.hpp"

namespace dai {
namespace gate {

std::vector<dai::StereoPair> getStereoPairs(dai::EepromData eepromData) {
    std::vector<dai::StereoPair> stereoPairs;
    if(eepromData.boardName.find("BK3389-VE") != std::string::npos) {  // Vermeer
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 10});
        stereoPairs.push_back(dai::StereoPair{
            .left = dai::CameraBoardSocket::CAM_B,
            .right = dai::CameraBoardSocket::CAM_D,
            .baseline = 4,
            .isVertical = true,
        });
    } else if(eepromData.boardName.find("BK3388-ME") != std::string::npos) {  // Meta
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
        stereoPairs.push_back(dai::StereoPair{
            .left = dai::CameraBoardSocket::CAM_B,
            .right = dai::CameraBoardSocket::CAM_D,
            .baseline = 3.5,
            .isVertical = true,
        });
    } else if(eepromData.boardName.find("DM3370") != std::string::npos) {  // Rae
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_D, .right = dai::CameraBoardSocket::CAM_E, .baseline = 7.5});
    } else if(eepromData.boardName.find("NG3396C") != std::string::npos) {  // Flashparking
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_A, .right = dai::CameraBoardSocket::CAM_D, .baseline = 3.5});
    } else if(eepromData.boardName.find("BK2316-GP") != std::string::npos) {  // Groopod
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    } else if(eepromData.boardName.find("DM9095") != std::string::npos) {  // OAK D PRO
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    } else if(eepromData.boardName.find("NG3398C-RA") != std::string::npos) {  // Rapyuta
        float baseline = 5;
        if(eepromData.boardRev.find("R3M1E1") != std::string::npos) {
            baseline = 4.5;
        }
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = baseline});
    } else if(eepromData.boardName.find("NG9498-ASM") != std::string::npos) {  // OAK4D
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    } else if(eepromData.boardName.find("PA9496") != std::string::npos) {  // OAK4-LITE
        stereoPairs.push_back(dai::StereoPair{.left = dai::CameraBoardSocket::CAM_B, .right = dai::CameraBoardSocket::CAM_C, .baseline = 7.5});
    }
    return stereoPairs;
}

}  // namespace gate
}  // namespace dai
