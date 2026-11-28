#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/// Camera sensor type
enum class Colormap : int32_t { NONE = 0, TURBO, JET, STEREO_TURBO, STEREO_JET };

}  // namespace dai
