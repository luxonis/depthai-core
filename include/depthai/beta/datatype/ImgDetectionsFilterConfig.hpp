#pragma once

// Compatibility header. ImgDetectionsFilterConfig moved out of the beta namespace in v3.12:
// include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp" and use dai::ImgDetectionsFilterConfig.
// Only the name is kept. The v3.12 config has different fields.

#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"

namespace dai {
namespace beta {

using ImgDetectionsFilterConfig [[deprecated("dai::beta::ImgDetectionsFilterConfig moved to dai::ImgDetectionsFilterConfig")]] = dai::ImgDetectionsFilterConfig;

}  // namespace beta
}  // namespace dai
