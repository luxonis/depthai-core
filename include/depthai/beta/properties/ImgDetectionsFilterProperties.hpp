#pragma once

// Compatibility header. ImgDetectionsFilterProperties moved out of the beta namespace in v3.12:
// include "depthai/properties/ImgDetectionsFilterProperties.hpp" and use dai::ImgDetectionsFilterProperties.

#include "depthai/properties/ImgDetectionsFilterProperties.hpp"

namespace dai {
namespace beta {

using ImgDetectionsFilterProperties [[deprecated("dai::beta::ImgDetectionsFilterProperties moved to dai::ImgDetectionsFilterProperties")]] =
    dai::ImgDetectionsFilterProperties;

}  // namespace beta
}  // namespace dai
