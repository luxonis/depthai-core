#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include "PointCloudGpu.hpp"
#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/node/PointCloud.hpp"
#include "depthai/properties/PointCloudProperties.hpp"

namespace dai {
namespace gate {

class PointCloud : public ::dai::node::PointCloud {
   public:
    constexpr static const char* NAME = node::PointCloud::NAME;

    PointCloud() : PointCloud(std::make_unique<dai::node::PointCloud::Properties>()) {}

    PointCloud(std::unique_ptr<PointCloudProperties> props) : dai::node::PointCloud(std::move(props)) {}

    bool runOnHost() const override {
        return true;  // Runs as a firmware thread (ARM CPU)
    }

   protected:  // createGpuBackend: null when there is no usable GPU; the node then computes on the CPU
    std::shared_ptr<node::internal::PointCloudGpuBackend> createGpuBackend(uint32_t gpuDevice, std::shared_ptr<spdlog::logger> logger) override {
        return createOpenClPointCloudBackend(gpuDevice, logger);
    }
};

}  // namespace gate
}  // namespace dai
