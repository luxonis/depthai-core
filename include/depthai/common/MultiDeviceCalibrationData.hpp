#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/CoordinateFrame.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * A single edge of the multi-device rig graph: the pose of `from` expressed with respect to `to`.
 *
 * `transform` follows the `Extrinsics` convention used everywhere else in depthai, i.e. it transforms points from the
 * `from` frame into the `to` frame. `transform.toCameraSocket` / `transform.toDeviceId` are kept consistent with `to`.
 */
struct RigEdge {
    /// Source frame of the edge.
    CoordinateFrame from;
    /// Target (reference) frame of the edge.
    CoordinateFrame to;
    /// Pose of `from` w.r.t. `to`.
    Extrinsics transform;
    /// Time the edge was estimated at, in microseconds since epoch. 0 means unknown.
    uint64_t timestamp = 0;
    /// Free-form origin of the edge, e.g. "dynamic-calibration", "cad".
    std::string source;

    DEPTHAI_SERIALIZE(RigEdge, from, to, transform, timestamp, source);
};

/**
 * Rig calibration of a multi-device setup: only the transformations *between* frames are stored. Per-device
 * calibration (intrinsics, distortion, intra-device extrinsics) always comes from the live device.
 *
 * The edges must form a forest - cycles are rejected. The graph may be disconnected, in which case transformations
 * are only defined within each connected component.
 */
struct MultiDeviceCalibrationData {
    /// Schema version of the data.
    uint32_t version = 1;
    /// Time the calibration was produced at, in microseconds since epoch. 0 means unknown.
    uint64_t timestamp = 0;
    /// Edges of the rig graph.
    std::vector<RigEdge> edges;
    /// Optional (alias, deviceId) pairs, letting a rig file be written against logical device names.
    std::vector<std::pair<std::string, std::string>> aliases;

    DEPTHAI_SERIALIZE(MultiDeviceCalibrationData, version, timestamp, edges, aliases);
};

}  // namespace dai
