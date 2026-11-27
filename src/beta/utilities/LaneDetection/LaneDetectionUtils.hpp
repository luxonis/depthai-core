#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "depthai/beta/datatype/Clusters.hpp"
#include "depthai/common/Point2f.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace LaneDetectionUtils {

std::vector<std::vector<Point2f>> decodeUfld(const std::vector<float>& values,  // Ultra-Fast-Lane-Detection decode; returns numLanes entries, empty lanes too
                                             const std::vector<std::size_t>& dims,
                                             const std::vector<std::int64_t>& rowAnchors,
                                             std::int64_t gridingNum,
                                             std::int64_t clsNumPerLane,
                                             std::uint32_t inputWidth,
                                             std::uint32_t inputHeight);

std::shared_ptr<Clusters> createClustersMessage(const std::vector<std::vector<Point2f>>& points);

}  // namespace LaneDetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
