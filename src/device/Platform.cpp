#include "depthai/device/Platform.hpp"

// std
#include <algorithm>
#include <stdexcept>

namespace dai {

std::string platform2string(Platform platform) {
    switch(platform) {
        case Platform::RVC2:
            return "RVC2";
        case Platform::RVC3:
            return "RVC3";
        case Platform::RVC4:
            return "RVC4";
    }
    throw std::invalid_argument("Unknown platform");
}

Platform string2platform(const std::string& platform) {
    // Convert to lower case
    std::string lower = platform;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    // Check which platform it is
    if(lower == "rvc2") {
        return Platform::RVC2;
    }
    if(lower == "rvc3") {
        return Platform::RVC3;
    }
    if(lower == "rvc4") {
        return Platform::RVC4;
    }

    throw std::invalid_argument("Unknown platform: " + platform);
}

}  // namespace dai
