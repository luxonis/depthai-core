#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/WorldConstraints.hpp"

namespace dai {
namespace utility {

/// Every marker a distance or a size of the constraints refers to, sorted, without duplicates.
std::vector<int32_t> referencedMarkerIds(const WorldConstraints& constraints);
/// Sockets referred to by the socket distances of the constraints, sorted, without duplicates.
std::vector<CameraBoardSocket> referencedSockets(const WorldConstraints& constraints);

}  // namespace utility
}  // namespace dai
