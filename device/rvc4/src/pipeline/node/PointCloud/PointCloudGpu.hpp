#pragma once

#include <cstdint>
#include <memory>

#include "depthai/pipeline/node/internal/PointCloudGpuBackend.hpp"

namespace spdlog {
class logger;
}

namespace dai {
namespace gate {

std::shared_ptr<node::internal::PointCloudGpuBackend> createOpenClPointCloudBackend(std::uint32_t gpuDevice, std::shared_ptr<spdlog::logger> logger);

}  // namespace gate
}  // namespace dai
