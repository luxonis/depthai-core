#pragma once

#include "depthai/beta/properties/ImgDetectionsFilterProperties.hpp"  // Compatibility header: moved out of beta in v3.12; the new node has another interface
#include "depthai/pipeline/node/ImgDetectionsFilter.hpp"

namespace dai {
namespace beta {
namespace node {

using ImgDetectionsFilter [[deprecated("dai::beta::node::ImgDetectionsFilter moved to dai::node::ImgDetectionsFilter")]] = dai::node::ImgDetectionsFilter;

}  // namespace node
}  // namespace beta
}  // namespace dai
