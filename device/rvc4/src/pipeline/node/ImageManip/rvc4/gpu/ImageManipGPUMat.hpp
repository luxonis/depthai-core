#pragma once

#include <CL/cl.h>
#include <spdlog/async_logger.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "ImageManipGPU.hpp"
#include "depthai/utility/OffsetMemory.hpp"

namespace dai::gate {

struct MatConfig {
    std::array<std::array<float, 3>, 3> matrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::size_t srcWidth = 0;
    std::size_t srcHeight = 0;
    std::size_t srcStrideBytes = 0;
    std::size_t dstWidth = 0;
    std::size_t dstHeight = 0;
    std::size_t dstStrideBytes = 0;
    std::uint32_t channels = 1;
    std::uint32_t bytesPerChannel = 1;
    std::array<std::uint16_t, 4> borderValue = {0, 0, 0, 0};

    bool operator==(const MatConfig& other) const {
        return matrix == other.matrix && srcWidth == other.srcWidth && srcHeight == other.srcHeight && srcStrideBytes == other.srcStrideBytes
               && dstWidth == other.dstWidth && dstHeight == other.dstHeight && dstStrideBytes == other.dstStrideBytes && channels == other.channels
               && bytesPerChannel == other.bytesPerChannel && borderValue == other.borderValue;
    }
};

class WarpGPUMat : public utility::WarpGPU<MatConfig> {
   public:
    WarpGPUMat() = default;
    WarpGPUMat(WarpGPUMat&&) = delete;
    WarpGPUMat& operator=(WarpGPUMat&&) = delete;
    WarpGPUMat(const WarpGPUMat&) = delete;
    WarpGPUMat& operator=(const WarpGPUMat&) = delete;
    ~WarpGPUMat() override;

    bool init(int numPlanes) override;
    bool buildPlane(int plane, const MatConfig& config) override;
    void enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) override;
    void releasePendingPlanes() override;

   private:
    using CachedBuffer = utility::CachedBuffer;

    struct PlaneBuffers {
        cl_mem inverseMatrixBuffer = nullptr;
        cl_mem borderValueBuffer = nullptr;
        size_t srcOffset = 0;
        size_t dstOffset = 0;
        std::unique_ptr<CachedBuffer> srcBuffer = nullptr;
        std::unique_ptr<CachedBuffer> dstBuffer = nullptr;
    };

    using PlaneData = utility::PlaneData<MatConfig, PlaneBuffers>;

    void releaseOpenClObjects(bool buffersOnly) override;

    std::vector<PlaneData> planeData;
};

}  // namespace dai::gate
