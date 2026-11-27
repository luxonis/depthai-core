#pragma once

#include <cstdint>
#include <memory>

#include "depthai/pipeline/node/internal/PointCloudGpuBackend.hpp"

namespace spdlog {
class logger;
}

namespace dai {
namespace gate {

/**
 * OpenCL implementation of the PointCloud deprojection for the RVC4 GPU, used by PointCloud::useGPU().
 * Returns null (after logging why) when the device has no usable OpenCL GPU; the node then falls back to the CPU.
 */
std::shared_ptr<node::internal::PointCloudGpuBackend> createOpenClPointCloudBackend(std::uint32_t gpuDevice, std::shared_ptr<spdlog::logger> logger);

}  // namespace gate
}  // namespace dai
