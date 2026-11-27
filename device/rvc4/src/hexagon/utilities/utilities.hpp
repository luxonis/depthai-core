#pragma once
// Public helper for the node code. Inline: the DSP offload libraries give only the C function
// dai_device_dsp_status_name (vendor/dai_device_dsp.h), no C++ type.

#include <string>

#include "vendor/dai_device_dsp.h"

namespace hexagon {

// Name of a FastRPC error code (the AEE_* codes of the Hexagon SDK), for log messages
inline std::string errorToString(int errorCode) {
    return std::string(dai_device_dsp_status_name(errorCode));
}

}  // namespace hexagon
