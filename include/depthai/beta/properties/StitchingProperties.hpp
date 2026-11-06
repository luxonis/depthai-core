#pragma once

// Compatibility header. StitchingProperties moved out of the beta namespace in v3.12:
// include "depthai/properties/StitchingProperties.hpp" and use dai::StitchingProperties.

#include "depthai/properties/StitchingProperties.hpp"

namespace dai {
namespace beta {

using StitchingProperties [[deprecated("dai::beta::StitchingProperties moved to dai::StitchingProperties")]] = dai::StitchingProperties;

}  // namespace beta
}  // namespace dai
