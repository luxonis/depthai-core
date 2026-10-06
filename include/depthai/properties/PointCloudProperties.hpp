#pragma once

#include <cstdint>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for PointCloud
 */
struct PointCloudProperties : PropertiesSerializable<Properties, PointCloudProperties> {
    /// How the points are computed (PointCloud::useCPU / useCPUMT / useGPU)
    enum class ComputeMethod : std::int32_t {
        CPU = 0,     ///< one thread per depth stream
        CPU_MT = 1,  ///< every depth stream split over `numThreads` threads
        GPU = 2      ///< GPU (Kompute on the host, the platform GPU backend on a device); falls back to CPU when unavailable
    };

    PointCloudConfig initialConfig;

    int numFramesPool = 4;

    ComputeMethod computeMethod = ComputeMethod::CPU;
    /// Threads per depth stream for ComputeMethod::CPU_MT
    std::uint32_t numThreads = 2;
    /// GPU device index for ComputeMethod::GPU
    std::uint32_t gpuDevice = 0;

    ~PointCloudProperties() override;
};

DEPTHAI_SERIALIZE_EXT(PointCloudProperties, initialConfig, numFramesPool, computeMethod, numThreads, gpuDevice);

}  // namespace dai