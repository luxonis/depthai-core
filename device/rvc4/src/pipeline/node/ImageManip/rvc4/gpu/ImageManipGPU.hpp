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

class CachedBuffer {
   public:
    cl_mem buffer = nullptr;
    std::size_t sizeBytes = 0;
    cl_mem_flags flags = 0;
    void* hostPtr = nullptr;
    int dmaBufFd = -1;
    bool usesExternalImport = false;
    std::weak_ptr<void> externalMemory;

    CachedBuffer() = default;
    CachedBuffer(CachedBuffer&&) = default;
    CachedBuffer& operator=(CachedBuffer&&) = default;
    CachedBuffer(const CachedBuffer&) = delete;
    CachedBuffer& operator=(const CachedBuffer&) = delete;

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
        if(requestedAllocation == nullptr) return false;  // Compare owners, not address and fd: those can be reused, a live control block cannot
        if(externalMemory.owner_before(requestedAllocation) || requestedAllocation.owner_before(externalMemory)) return false;
        return hostPtr == requestedHostPtr && dmaBufFd == requestedDmaBufFd;      // zero-copy: must be the same DMA buffer
    }

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

template <typename Config, typename PlaneBuffers>
struct PlaneData {
    Config config{};
    PlaneBuffers buffers{};
    bool enqueued = false;
};

struct DeviceBufferBinding {
    cl_mem buffer = nullptr;
    bool requiresUpload = false;
    bool requiresDownload = false;
    cl_int offsetBytes = 0;
};

class ClExecutionError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
struct ExternalMemoryInfo {
    int dmaBufFd = -1;
    void* hostPtr = nullptr;
    cl_uint hostCachePolicy = CL_MEM_HOST_WRITEBACK_QCOM;
    std::shared_ptr<void> allocationOwner;
};

std::optional<ExternalMemoryInfo> getExternalMemoryInfo(const std::shared_ptr<OffsetMemory>& memory);
#endif

std::size_t getRequiredImageBytes(std::size_t width, std::size_t height, std::size_t strideBytes, std::size_t pixelStrideBytes);

std::size_t getMaxBufferSize(cl_device_id device);

void validateBufferSize(cl_device_id device, std::size_t sizeBytes, const char* what);

void checkStatus(cl_int status, const char* what);

std::string getDeviceExtensions(cl_device_id device);

bool hasExtensionToken(const std::string& extensionList, const char* extension);

std::string getBuildLog(cl_program builtProgram, cl_device_id builtDevice);

void logError(const std::shared_ptr<spdlog::async_logger>& logger, const std::exception& error);

void requireInitialized(bool initialized, const char* message);

std::size_t validatePlaneIndex(int plane, std::size_t planeCount, const char* negativeMessage, const char* outOfBoundsMessage);

template <typename Config>  // Call order: init(), buildPlane(), enqueueWarpPlane() at most once per plane, then warp()
class WarpGPU {
   public:
    WarpGPU() = default;
    WarpGPU(WarpGPU&&) = delete;
    WarpGPU& operator=(WarpGPU&&) = delete;
    WarpGPU(const WarpGPU&) = delete;
    WarpGPU& operator=(const WarpGPU&) = delete;

    virtual bool init(int numPlanes) = 0;
    virtual bool buildPlane(int plane, const Config& config) = 0;
    virtual void enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) = 0;
    virtual void releasePendingPlanes() = 0;

    virtual ~WarpGPU() = default;

    void warp() {
        requireInitialized(initialized, "WarpGPU::init must be called before WarpGPU::warp");

        try {
            checkStatus(clFinish(queue), "clFinish");
        } catch(...) {
            if(clFinish(queue) != CL_SUCCESS) std::terminate();  // A second failure is fatal: unwinding could free GPU memory still in use
            releasePendingPlanes();
            throw;
        }

        releasePendingPlanes();
    }

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
