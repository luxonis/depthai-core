#pragma once

// Compatibility header. ImgDetectionsFilter moved out of the beta namespace in v3.12:
// include "depthai/pipeline/node/ImgDetectionsFilter.hpp" and use dai::node::ImgDetectionsFilter.
// Only the name is kept. The v3.12 node has a different interface (inputs, out, new config fields).

#include "depthai/beta/properties/ImgDetectionsFilterProperties.hpp"
#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"

namespace dai {
namespace beta {
namespace node {

using ImgDetectionsFilter [[deprecated("dai::beta::node::ImgDetectionsFilter moved to dai::node::ImgDetectionsFilter")]] = dai::node::ImgDetectionsFilter;

}  // namespace node
}  // namespace beta
}  // namespace dai
