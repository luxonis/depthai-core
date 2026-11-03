#pragma once

// Compatibility header. Stitching moved out of the beta namespace in v3.12:
// include "depthai/pipeline/node/Stitching.hpp" and use dai::node::Stitching.

#include "depthai/beta/properties/StitchingProperties.hpp"
#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace beta {
namespace node {

using Stitching [[deprecated("dai::beta::node::Stitching moved to dai::node::Stitching")]] = dai::node::Stitching;

}  // namespace node
}  // namespace beta
}  // namespace dai
