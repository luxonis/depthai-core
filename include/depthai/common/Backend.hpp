#pragma once

#include "depthai/properties/ImageManipProperties.hpp"

namespace dai {

/**
 * Execution backend preference shared by processing nodes.
 * CPU uses the host or device CPU, HW selects dedicated hardware when supported,
 * GPU selects graphics compute, and AUTO lets the node select a backend.
 * Supported backends and fallback behavior are documented by each node.
 * This aliases the original ImageManip enum to preserve its C++ type identity and ABI.
 */
using Backend = ImageManipProperties::Backend;

}  // namespace dai
