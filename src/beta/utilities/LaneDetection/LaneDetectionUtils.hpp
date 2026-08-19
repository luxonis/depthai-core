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

/**
 * @brief Decode lane points from an Ultra-Fast-Lane-Detection (UFLD) output tensor.
 *
 * The tensor must be a 4D tensor of shape (batch, gridingNum + 1, clsNumPerLane, numLanes),
 * stored row-major over dims; the first entry of the batch dimension is decoded. Per row anchor
 * and lane, a softmax over the first gridingNum griding entries (in reversed row-anchor order)
 * yields the expected griding location as sum(probability * (gridIndex + 1)); locations whose
 * arg-max over all gridingNum + 1 entries selects the last entry (the no-lane class) are
 * suppressed to 0. Lanes with more than 2 non-suppressed locations produce one point per
 * positive location: x = int(location * (inputWidth - 1) / (gridingNum - 1)) - 1 and
 * y = int(inputHeight * (rowAnchors[clsNumPerLane - 1 - k] / inputHeight)) - 1 with the integer
 * truncation of the source implementation, both normalized by the input (width, height). Lanes
 * failing the check stay empty but remain in the result, so the result always contains numLanes
 * entries.
 *
 * @param values Output tensor values, stored row-major over dims.
 * @param dims Output tensor shape, (batch, gridingNum + 1, clsNumPerLane, numLanes).
 * @param rowAnchors Row anchors; must contain at least clsNumPerLane entries.
 * @param gridingNum Griding number, must be greater than 1.
 * @param clsNumPerLane Number of points per lane, must be greater than 0.
 * @param inputWidth Model input image width the points are normalized by.
 * @param inputHeight Model input image height the points are normalized by.
 * @return Per-lane vectors of normalized points, one entry per lane column.
 */
std::vector<std::vector<Point2f>> decodeUfld(const std::vector<float>& values,
                                             const std::vector<std::size_t>& dims,
                                             const std::vector<std::int64_t>& rowAnchors,
                                             std::int64_t gridingNum,
                                             std::int64_t clsNumPerLane,
                                             std::uint32_t inputWidth,
                                             std::uint32_t inputHeight);

/**
 * @brief Create a Clusters message from per-lane point lists.
 *
 * Every lane produces one cluster, including empty lanes. Clusters are labeled sequentially
 * from 0 in lane order.
 *
 * @param points Per-lane vectors of points.
 * @return Clusters message with one labeled cluster per lane.
 */
std::shared_ptr<Clusters> createClustersMessage(const std::vector<std::vector<Point2f>>& points);

}  // namespace LaneDetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
