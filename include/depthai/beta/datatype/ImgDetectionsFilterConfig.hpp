#pragma once

#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"  // Compatibility header: moved out of beta in v3.12; the new config has different fields

namespace dai {
namespace beta {

using ImgDetectionsFilterConfig [[deprecated("dai::beta::ImgDetectionsFilterConfig moved to dai::ImgDetectionsFilterConfig")]] = dai::ImgDetectionsFilterConfig;

}  // namespace beta
}  // namespace dai
