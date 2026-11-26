#pragma once

#include <CL/cl.h>
#include <CL/cl_ext.h>
#if defined(__has_include)
    #if __has_include(<CL/cl_ext_qcom.h>)
        #include <CL/cl_ext_qcom.h>
        #define DEPTHAI_HAVE_CL_EXT_QCOM 1
    #endif
#endif

#include <spdlog/async_logger.h>

#include <cstddef>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "depthai/utility/OffsetMemory.hpp"

namespace dai::gate::utility {

/**
 * @brief RAII wrapper for an OpenCL buffer and the metadata needed to reuse it.
 */
class CachedBuffer {
   public:
    cl_mem buffer = nullptr;
    std::size_t sizeBytes = 0;
    cl_mem_flags flags = 0;
    void* hostPtr = nullptr;
    int dmaBufFd = -1;
    bool usesExternalImport = false;
    // Track the backing allocation, rather than a particular offset view, without owning it.
    std::weak_ptr<void> externalMemory;

    CachedBuffer() = default;
    CachedBuffer(CachedBuffer&&) = default;
    CachedBuffer& operator=(CachedBuffer&&) = default;
    CachedBuffer(const CachedBuffer&) = delete;
    CachedBuffer& operator=(const CachedBuffer&) = delete;

    /**
     * @brief Returns true when this cached buffer can be bound for a request with the given parameters.
     *
     * A staged buffer only has to match in shape. A zero-copy import is a view of one particular DMA
     * allocation, so it must share ownership with the incoming allocation and match its mapping and fd.
     */
    bool matches(std::size_t requestedBytes,
                 cl_mem_flags requestedFlags,
                 bool externalImport,
                 const void* requestedHostPtr,
                 int requestedDmaBufFd,
                 const std::shared_ptr<void>& requestedAllocation) const {
        if(buffer == nullptr) return false;                                       // nothing cached
        if(sizeBytes != requestedBytes || flags != requestedFlags) return false;  // different shape
        if(usesExternalImport != externalImport) return false;                    // different kind of buffer
        if(!externalImport) return true;                                          // staged copy: any host memory fits
        if(requestedAllocation == nullptr) return false;
        // The incoming shared token keeps the allocation alive. Control-block identity cannot be
        // recycled while our weak token exists, even if the old address and fd have been reused.
        if(externalMemory.owner_before(requestedAllocation) || requestedAllocation.owner_before(externalMemory)) return false;
        return hostPtr == requestedHostPtr && dmaBufFd == requestedDmaBufFd;      // zero-copy: must be the same DMA buffer
    }

    /**
     * @brief Releases the cached OpenCL buffer and clears the cached metadata.
     */
    void release() {
        if(buffer != nullptr) {
            clReleaseMemObject(buffer);
            buffer = nullptr;
        }
        externalMemory.reset();
        sizeBytes = 0;
        flags = 0;
        hostPtr = nullptr;
        dmaBufFd = -1;
        usesExternalImport = false;
    }
    ~CachedBuffer() {
        release();
    }
};

/**
 * @brief Per-plane cache entry storing the active configuration, buffers, and enqueue state.
 *
 * @tparam Config Plane configuration type.
 * @tparam PlaneBuffers Plane-specific OpenCL buffer bundle.
 */
template <typename Config, typename PlaneBuffers>
struct PlaneData {
    Config config{};
    PlaneBuffers buffers{};
    bool enqueued = false;
};

/**
 * @brief Describes how an image plane is currently bound to an OpenCL buffer.
 */
struct DeviceBufferBinding {
    cl_mem buffer = nullptr;
    bool requiresUpload = false;
    bool requiresDownload = false;
    cl_int offsetBytes = 0;
};

/**
 * @brief Exception thrown when an OpenCL operation fails validation or execution.
 */
class ClExecutionError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
/**
 * @brief External-memory import parameters used for Qualcomm zero-copy OpenCL buffers.
 */
struct ExternalMemoryInfo {
    int dmaBufFd = -1;
    void* hostPtr = nullptr;
    cl_uint hostCachePolicy = CL_MEM_HOST_WRITEBACK_QCOM;
    std::shared_ptr<void> allocationOwner;
};

/**
 * @brief Extracts DMA-BUF backed import information from an `OffsetMemory` instance when available.
 *
 * @param memory Source memory wrapper to inspect.
 * @return Import metadata for OpenCL external-memory extensions, or `std::nullopt` when the memory
 *         cannot be imported directly.
 */
std::optional<ExternalMemoryInfo> getExternalMemoryInfo(const std::shared_ptr<OffsetMemory>& memory);
#endif

/**
 * @brief Computes the minimum contiguous byte count needed to address an image plane.
 *
 * This accounts for row stride and pixel stride, so the result may be smaller than
 * `height * strideBytes` when the last row is only partially used.
 *
 * @param width Plane width in pixels.
 * @param height Plane height in pixels.
 * @param strideBytes Distance in bytes between the start of consecutive rows.
 * @param pixelStrideBytes Number of bytes occupied by one pixel.
 * @return Minimum number of bytes the backing buffer must expose.
 */
std::size_t getRequiredImageBytes(std::size_t width, std::size_t height, std::size_t strideBytes, std::size_t pixelStrideBytes);

/**
 * @brief Queries the maximum single OpenCL allocation supported by a device.
 *
 * @param device OpenCL device to query.
 * @return Maximum allocatable buffer size in bytes, clamped to `std::size_t`.
 */
std::size_t getMaxBufferSize(cl_device_id device);

/**
 * @brief Verifies that a requested OpenCL buffer size is non-zero and supported by the device.
 *
 * @param device OpenCL device that will own the allocation.
 * @param sizeBytes Requested allocation size in bytes.
 * @param what Context string used in the thrown error.
 * @throws ClExecutionError If the requested size is zero or exceeds device limits.
 */
void validateBufferSize(cl_device_id device, std::size_t sizeBytes, const char* what);

/**
 * @brief Throws when an OpenCL API call did not return `CL_SUCCESS`.
 *
 * @param status Status code returned by the OpenCL call.
 * @param what Context string used in the thrown error.
 * @throws ClExecutionError If `status` is not `CL_SUCCESS`.
 */
void checkStatus(cl_int status, const char* what);

/**
 * @brief Returns the space-delimited OpenCL extension list reported by a device.
 *
 * @param device OpenCL device to query.
 * @return Device extension string without the trailing null terminator.
 */
std::string getDeviceExtensions(cl_device_id device);

/**
 * @brief Tests whether an extension name exists as a full token in an extension list.
 *
 * @param extensionList Space-delimited extension string.
 * @param extension Extension token to search for.
 * @return `true` when the exact token is present; otherwise `false`.
 */
bool hasExtensionToken(const std::string& extensionList, const char* extension);

/**
 * @brief Retrieves the compiler log from an OpenCL program build.
 *
 * @param builtProgram Program object that was built.
 * @param builtDevice Device used for the build.
 * @return Build log text, or an empty string when no log is available.
 */
std::string getBuildLog(cl_program builtProgram, cl_device_id builtDevice);

/**
 * @brief Logs an exception through the configured async logger or `stderr` as a fallback.
 *
 * @param logger Optional logger instance.
 * @param error Exception to report.
 */
void logError(const std::shared_ptr<spdlog::async_logger>& logger, const std::exception& error);

/**
 * @brief Throws when a GPU warp object has not completed initialization.
 *
 * @param initialized Current initialization state.
 * @param message Error message used if initialization is missing.
 */
void requireInitialized(bool initialized, const char* message);

/**
 * @brief Validates a plane index and returns it as `std::size_t`.
 *
 * @param plane Plane index supplied by the caller.
 * @param planeCount Number of configured planes.
 * @param negativeMessage Error message used when `plane` is negative.
 * @param outOfBoundsMessage Error message used when `plane` is outside `[0, planeCount)`.
 * @return Validated plane index.
 */
std::size_t validatePlaneIndex(int plane, std::size_t planeCount, const char* negativeMessage, const char* outOfBoundsMessage);

/**
 * @brief Common OpenCL warp interface shared by matrix- and mesh-based GPU image manipulation backends.
 *
 * The expected call order is `init()`, `buildPlane()` for each plane that should be configured,
 * `enqueueWarpPlane()` for each plane that should run, and finally `warp()` to wait for completion
 * and release per-dispatch resources.
 *
 * @tparam Config Per-plane configuration type consumed by the backend.
 */
template <typename Config>
class WarpGPU {
   public:
    WarpGPU() = default;
    WarpGPU(WarpGPU&&) = delete;
    WarpGPU& operator=(WarpGPU&&) = delete;
    WarpGPU(const WarpGPU&) = delete;
    WarpGPU& operator=(const WarpGPU&) = delete;

    /**
     * @brief Initializes the OpenCL backend and allocates per-plane bookkeeping.
     *
     * @param numPlanes Number of planes that will be configured and dispatched.
     * @return `true` when the backend is ready to use, otherwise `false`.
     */
    virtual bool init(int numPlanes) = 0;
    /**
     * @brief Builds or refreshes the cached OpenCL resources for one plane configuration.
     *
     * Implementations may reuse already-created buffers when the configuration is unchanged.
     *
     * @param plane Plane index to configure.
     * @param config Plane configuration to cache.
     * @return `true` when the plane is ready for `enqueueWarpPlane()`, otherwise `false`.
     */
    virtual bool buildPlane(int plane, const Config& config) = 0;
    /**
     * @brief Enqueues one configured plane for execution.
     *
     * The plane must have been successfully prepared with `buildPlane()` and cannot be enqueued
     * again until `warp()` completes and clears the pending state.
     *
     * @param plane Plane index to dispatch.
     * @param src Source image memory.
     * @param dst Destination image memory.
     */
    virtual void enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) = 0;
    /**
     * @brief Releases temporary per-dispatch resources for any planes that were enqueued.
     */
    virtual void releasePendingPlanes() = 0;

    virtual ~WarpGPU() = default;

    /**
     * @brief Waits for all queued plane warps to finish and clears pending plane state.
     *
     * @throws std::runtime_error If `init()` has not been called.
     * @throws ClExecutionError If `clFinish()` fails.
     */
    void warp() {
        requireInitialized(initialized, "WarpGPU::init must be called before WarpGPU::warp");

        try {
            checkStatus(clFinish(queue), "clFinish");
        } catch(...) {
            // Keep the caller's buffers alive until completion is established.
            // A second failure is fatal: unwinding could free active GPU memory.
            if(clFinish(queue) != CL_SUCCESS) std::terminate();
            releasePendingPlanes();
            throw;
        }

        releasePendingPlanes();
    }

    /**
     * @brief Installs the logger used by helper methods and derived implementations.
     *
     * @param logger Async logger to use for error reporting.
     */
    void setLogger(std::shared_ptr<spdlog::async_logger> logger) {
        this->logger = std::move(logger);
    }

   protected:
    virtual void releaseOpenClObjects(bool buffersOnly) = 0;

    bool initialized = false;

    cl_platform_id platform = nullptr;
    cl_device_id device = nullptr;
    cl_context context = nullptr;
    cl_command_queue queue = nullptr;
    cl_program program = nullptr;
    cl_kernel kernel = nullptr;
    bool qcomIoCoherentHostPtrSupported = false;
    bool qcomDmabufHostPtrSupported = false;
    bool qcomIonHostPtrSupported = false;

    std::unique_ptr<CachedBuffer> fullSrcBuffer = nullptr;  // Used for zero-copy
    std::unique_ptr<CachedBuffer> fullDstBuffer = nullptr;  // Used for zero-copy

    std::shared_ptr<spdlog::async_logger> logger;
};

}  // namespace dai::gate::utility
