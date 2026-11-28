#pragma once

#include <string>

#include "vendor/dai_device_dsp.h"

namespace hexagon {

inline std::string errorToString(int errorCode) {
    return std::string(dai_device_dsp_status_name(errorCode));
}

}  // namespace hexagon
