#pragma once

#include "depthai/beta/properties/StitchingProperties.hpp"
#include "depthai/pipeline/node/Stitching.hpp"

namespace dai {
namespace beta {
namespace node {

using Stitching [[deprecated("dai::beta::node::Stitching moved to dai::node::Stitching")]] = dai::node::Stitching;

}  // namespace node
}  // namespace beta
}  // namespace dai
