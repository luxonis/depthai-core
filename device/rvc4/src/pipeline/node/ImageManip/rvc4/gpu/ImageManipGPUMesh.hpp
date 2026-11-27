#pragma once

#include <CL/cl.h>
#include <spdlog/async_logger.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "ImageManipGPU.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/utility/OffsetMemory.hpp"

namespace dai::gate {

struct MeshConfig {
    std::shared_ptr<const std::vector<float>> meshX;
    std::shared_ptr<const std::vector<float>> meshY;
    std::size_t meshWidth = 0;
    std::size_t meshHeight = 0;
    std::size_t srcWidth = 0;
    std::size_t srcHeight = 0;
    std::size_t srcStrideBytes = 0;
    std::size_t dstWidth = 0;
    std::size_t dstHeight = 0;
    std::size_t dstStrideBytes = 0;
    std::uint32_t channels = 1;
    std::uint32_t bytesPerChannel = 1;
    std::array<std::uint16_t, 4> borderValue = {0, 0, 0, 0};

    bool operator==(const MeshConfig& other) const {
        if(meshWidth != other.meshWidth || meshHeight != other.meshHeight || srcWidth != other.srcWidth || srcHeight != other.srcHeight
           || srcStrideBytes != other.srcStrideBytes || dstWidth != other.dstWidth || dstHeight != other.dstHeight || dstStrideBytes != other.dstStrideBytes
           || channels != other.channels || bytesPerChannel != other.bytesPerChannel || borderValue != other.borderValue || meshX.get() != other.meshX.get()
           || meshY.get() != other.meshY.get()) {
            return false;
        }
        return true;
    }
};

class WarpGPUMesh : public utility::WarpGPU<MeshConfig> {
   public:
    WarpGPUMesh() = default;
    WarpGPUMesh(WarpGPUMesh&&) = delete;
    WarpGPUMesh& operator=(WarpGPUMesh&&) = delete;
    WarpGPUMesh(const WarpGPUMesh&) = delete;
    WarpGPUMesh& operator=(const WarpGPUMesh&) = delete;
    ~WarpGPUMesh() override;

    static std::vector<Point2f> getMeshFromMatrix(
        const std::array<std::array<float, 3>, 3>& matrix, std::size_t meshWidth, std::size_t meshHeight, std::size_t dstWidth, std::size_t dstHeight);

    bool init(int numPlanes) override;
    bool buildPlane(int plane, const MeshConfig& config) override;
    void enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) override;
    void releasePendingPlanes() override;

   private:
    using CachedBuffer = utility::CachedBuffer;

    struct PlaneBuffers {
        cl_mem meshXBuffer = nullptr;
        cl_mem meshYBuffer = nullptr;
        cl_mem borderValueBuffer = nullptr;
        size_t srcOffset = 0;
        size_t dstOffset = 0;
        std::unique_ptr<CachedBuffer> srcBuffer = nullptr;
        std::unique_ptr<CachedBuffer> dstBuffer = nullptr;
    };

    using PlaneData = utility::PlaneData<MeshConfig, PlaneBuffers>;

    void releaseOpenClObjects(bool buffersOnly) override;

    cl_kernel denseKernel = nullptr;
    std::vector<PlaneData> planeData;
};

}  // namespace dai::gate
