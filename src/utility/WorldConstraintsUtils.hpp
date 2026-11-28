#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/WorldConstraints.hpp"

namespace dai {
namespace utility {

std::vector<int32_t> referencedMarkerIds(const WorldConstraints& constraints);          // Sorted, without duplicates
std::vector<CameraBoardSocket> referencedSockets(const WorldConstraints& constraints);  // Sorted, without duplicates

}  // namespace utility
}  // namespace dai
