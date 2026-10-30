#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/utility/Memory.hpp"

namespace dai {
namespace node {

/**
 * @internal
 * Platform specific GPU implementation of the dense deprojection used by the PointCloud node, for example
 * OpenCL on RVC4 devices. A platform node class provides it by overriding PointCloud::createGpuBackend;
 * PointCloud::useGPU then computes through it. Without a backend the built-in Kompute path is used when
 * compiled in, otherwise the node falls back to the CPU.
 * The original dai::node::PointCloudGpuBackend type is retained for source and ABI compatibility.
 * This is an SDK/firmware implementation interface, not a user extension API.
 * Applications select GPU computation with PointCloud::useGPU().
 */
class PointCloudGpuBackend {
   public:
    /// Inputs of the deprojection that stay constant between frames of the same size, intrinsics and target
    struct Geometry {
        unsigned int width = 0;
        unsigned int height = 0;
        /// Multiplier from the raw uint16 depth value (millimeters) to the output length unit
        float depthScale = 1.0f;
        /// Undistorted normalized ray (x/z, y/z) of every pixel, width*height entries, row-major
        const Point2f* rays = nullptr;
        /// Changes whenever the content of `rays` changes, so that a backend can cache the uploaded table
        std::uint64_t raysVersion = 0;
        /// Apply `transform` (4x4 row-major, R*p + t) to every valid point (z > 0)
        bool hasTransform = false;
        std::array<std::array<float, 4>, 4> transform{};
    };

    virtual ~PointCloudGpuBackend() = default;

    /**
     * Deproject a RAW16 depth frame into width*height points: z = depth * depthScale, x = ray.x * z, y = ray.y * z,
     * invalid depth (0) gives (0, 0, 0); valid points are transformed when `hasTransform` is set.
     * @param depthMemory Memory object that owns `depthData` when known (allows zero-copy import), may be null
     * @returns The dense points in memory owned by the backend (for example a mapped GPU buffer), valid until the
     * next compute call on this backend. The caller copies or compacts them into its own buffers.
     */
    virtual const Point3f* computeDense(const Geometry& geometry, const std::uint8_t* depthData, const std::shared_ptr<Memory>& depthMemory) = 0;

    /**
     * Same as computeDense with the RGB888i color of every pixel copied into the point (alpha = 255).
     */
    virtual const Point3fRGBA* computeDenseColored(const Geometry& geometry,
                                                   const std::uint8_t* depthData,
                                                   const std::shared_ptr<Memory>& depthMemory,
                                                   const std::uint8_t* colorData,
                                                   const std::shared_ptr<Memory>& colorMemory) = 0;
};

namespace internal {
// Keep the internal spelling introduced after the original public type.
using PointCloudGpuBackend = ::dai::node::PointCloudGpuBackend;
}  // namespace internal
}  // namespace node
}  // namespace dai
