#pragma once
#include <cstdint>
#include <optional>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Placement and priority of the threads the dynamic calibration library computes on.
 *
 * The library runs its work on threads it owns (scheduler workers and a pool for its OpenCV loops), named
 * "dcl-worker". Those threads are process-wide: the policy set on one node reaches every DynamicCalibration and
 * AutoCalibration node running in the same process, the device firmware or the host application. A field left
 * empty keeps the library default.
 */
struct DynamicCalibrationThreadPolicy {
    /// Scheduler workers, at least 1. Library default 2.
    std::optional<int32_t> schedulerThreads;
    /// Threads one OpenCV loop may use, the calling worker included, at least 1. Library default min(8, CPUs).
    /// At most schedulerThreads - 1 + openCvThreads library threads compute at the same time.
    std::optional<int32_t> openCvThreads;
    /// Linux nice value, -20 (most urgent) to 19 (least urgent). Default: inherited from the process.
    std::optional<int32_t> niceness;
    /// CPUs the threads may run on; an empty list allows every CPU. Default: inherited from the process.
    std::optional<std::vector<uint32_t>> cpuAffinity;

    DEPTHAI_SERIALIZE(DynamicCalibrationThreadPolicy, schedulerThreads, openCvThreads, niceness, cpuAffinity);
};

/**
 * Specify properties for Dynamic calibration.
 */

struct DynamicCalibrationProperties : PropertiesSerializable<Properties, DynamicCalibrationProperties> {
    bool emptyBool;
    /// Thread policy handed to the dynamic calibration library when the node starts.
    DynamicCalibrationThreadPolicy threadPolicy;

    ~DynamicCalibrationProperties() override;
};

DEPTHAI_SERIALIZE_EXT(DynamicCalibrationProperties, emptyBool, threadPolicy);

}  // namespace dai
