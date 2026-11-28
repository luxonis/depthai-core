#include "PointCloudGpu.hpp"

#include <CL/cl.h>
#include <CL/cl_ext.h>
#if defined(__has_include)
    #if __has_include(<CL/cl_ext_qcom.h>)
        #include <CL/cl_ext_qcom.h>
    #endif
#endif

#include <spdlog/logger.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "depthai/pipeline/node/internal/PointCloudGpuBackend.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "pipeline/node/ImageManip/rvc4/gpu/ImageManipGPU.hpp"
#include "utilities/DmaHeapMemory.hpp"

namespace dai {
namespace gate {

namespace {

constexpr const char* kKernelSource = R"CLC(
__kernel void depth_to_points(__global const ushort* depth,
                              const int depthOffset,
                              __global const float2* rays,
                              __global float* points,
                              const float scale,
                              const int count,
                              const int applyTransform,
                              __global const float* transform) {
    const int i = get_global_id(0);
    if(i >= count) return;
    const float z0 = (float)depth[depthOffset + i] * scale;
    float x = 0.0f, y = 0.0f, z = z0;
    if(z0 > 0.0f) {
        const float2 r = rays[i];
        x = r.x * z0;
        y = r.y * z0;
        if(applyTransform) {
            const float tx = transform[0] * x + transform[1] * y + transform[2] * z0 + transform[3];
            const float ty = transform[4] * x + transform[5] * y + transform[6] * z0 + transform[7];
            const float tz = transform[8] * x + transform[9] * y + transform[10] * z0 + transform[11];
            x = tx;
            y = ty;
            z = tz;
        }
    }
    points[3 * i] = x;
    points[3 * i + 1] = y;
    points[3 * i + 2] = z;
}

__kernel void depth_to_points_colored(__global const ushort* depth,
                                      const int depthOffset,
                                      __global const uchar* color,
                                      const int colorOffset,
                                      __global const float2* rays,
                                      __global float4* points,
                                      const float scale,
                                      const int count,
                                      const int applyTransform,
                                      __global const float* transform) {
    const int i = get_global_id(0);
    if(i >= count) return;
    const float z0 = (float)depth[depthOffset + i] * scale;
    float x = 0.0f, y = 0.0f, z = z0;
    if(z0 > 0.0f) {
        const float2 r = rays[i];
        x = r.x * z0;
        y = r.y * z0;
        if(applyTransform) {
            const float tx = transform[0] * x + transform[1] * y + transform[2] * z0 + transform[3];
            const float ty = transform[4] * x + transform[5] * y + transform[6] * z0 + transform[7];
            const float tz = transform[8] * x + transform[9] * y + transform[10] * z0 + transform[11];
            x = tx;
            y = ty;
            z = tz;
        }
    }
    const int c = colorOffset + 3 * i;
    const uchar4 rgba = (uchar4)(color[c], color[c + 1], color[c + 2], (uchar)255);
    points[i] = (float4)(x, y, z, as_float(rgba));
}
)CLC";

#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
    #define POINTCLOUD_GPU_ZERO_COPY 1
#else
    #define POINTCLOUD_GPU_ZERO_COPY 0
#endif

struct SharedContext {
    cl_platform_id platform = nullptr;
    cl_device_id device = nullptr;
    cl_context context = nullptr;
    cl_program program = nullptr;
    bool ioCoherentHostPtr = false;
    bool dmabufHostPtr = false;
    bool ionHostPtr = false;

    SharedContext() {
        cl_int status = CL_SUCCESS;
        utility::checkStatus(clGetPlatformIDs(1, &platform, nullptr), "clGetPlatformIDs");
        utility::checkStatus(clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr), "clGetDeviceIDs(GPU)");
        const std::string extensions = utility::getDeviceExtensions(device);
        ioCoherentHostPtr = utility::hasExtensionToken(extensions, "cl_qcom_ext_host_ptr_iocoherent");
#if POINTCLOUD_GPU_ZERO_COPY
        const bool extHostPtr = utility::hasExtensionToken(extensions, "cl_qcom_ext_host_ptr");
    #if defined(CL_MEM_DMABUF_HOST_PTR_QCOM)
        dmabufHostPtr = extHostPtr && utility::hasExtensionToken(extensions, "cl_qcom_dmabuf_host_ptr");
    #endif
    #if defined(CL_MEM_ION_HOST_PTR_QCOM)
        ionHostPtr = extHostPtr && utility::hasExtensionToken(extensions, "cl_qcom_ion_host_ptr");
    #endif
#endif
        context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &status);
        utility::checkStatus(status, "clCreateContext");
        const char* source = kKernelSource;
        program = clCreateProgramWithSource(context, 1, &source, nullptr, &status);
        utility::checkStatus(status, "clCreateProgramWithSource");
        status = clBuildProgram(program, 1, &device, nullptr, nullptr, nullptr);
        if(status != CL_SUCCESS) {
            std::ostringstream error;
            error << "clBuildProgram failed with OpenCL error " << status;
            const std::string buildLog = utility::getBuildLog(program, device);
            if(!buildLog.empty()) error << '\n' << buildLog;
            throw utility::ClExecutionError(error.str());
        }
    }

    static std::shared_ptr<SharedContext> get() {  // Process lifetime: the program outlives the nodes; a throwing construction is retried on the next call
        static const std::shared_ptr<SharedContext> shared = std::make_shared<SharedContext>();
        return shared;
    }
};

struct ReusableBuffer {
    cl_mem buffer = nullptr;
    std::size_t sizeBytes = 0;
    cl_mem_flags flags = 0;
    const void* hostPtr = nullptr;  // zero-copy imports only
    int dmaBufFd = -1;

    ReusableBuffer() = default;
    ReusableBuffer(const ReusableBuffer&) = delete;
    ReusableBuffer& operator=(const ReusableBuffer&) = delete;
    ~ReusableBuffer() {
        release();
    }
    void release() {
        if(buffer != nullptr) clReleaseMemObject(buffer);
        buffer = nullptr;
        sizeBytes = 0;
        flags = 0;
        hostPtr = nullptr;
        dmaBufFd = -1;
    }
    bool matches(std::size_t bytes, cl_mem_flags wantedFlags, const void* wantedHostPtr) const {
        return buffer != nullptr && sizeBytes == bytes && flags == wantedFlags && hostPtr == wantedHostPtr;
    }
};

struct InputBinding {
    cl_mem buffer = nullptr;
    cl_int offsetBytes = 0;
};

class OpenClPointCloudBackend : public node::internal::PointCloudGpuBackend {
   public:
    OpenClPointCloudBackend(std::uint32_t gpuDevice, std::shared_ptr<spdlog::logger> log) : shared(SharedContext::get()), logger(std::move(log)) {
        (void)gpuDevice;  // the RVC4 has a single GPU
        cl_int status = CL_SUCCESS;
#if CL_TARGET_OPENCL_VERSION >= 200
        const cl_queue_properties queueProperties[] = {CL_QUEUE_PROPERTIES, CL_QUEUE_PROFILING_ENABLE, 0};
        queue = clCreateCommandQueueWithProperties(shared->context, shared->device, queueProperties, &status);
#else
        queue = clCreateCommandQueue(shared->context, shared->device, CL_QUEUE_PROFILING_ENABLE, &status);
#endif
        utility::checkStatus(status, "clCreateCommandQueue");
        kernel = clCreateKernel(shared->program, "depth_to_points", &status);
        utility::checkStatus(status, "clCreateKernel(depth_to_points)");
        coloredKernel = clCreateKernel(shared->program, "depth_to_points_colored", &status);
        utility::checkStatus(status, "clCreateKernel(depth_to_points_colored)");
        selfTest();
        if(logger) {
            logger->info("PointCloud GPU backend ready (OpenCL{}{}, {} output)",
                         shared->dmabufHostPtr ? ", dma-buf import" : (shared->ionHostPtr ? ", ION import" : ", staged copies"),
                         shared->ioCoherentHostPtr ? ", IO-coherent" : "",
                         outputMemory ? (outputIoCoherent ? "IO-coherent dma-buf" : "cached dma-buf") : "driver allocated");
        }
    }

    ~OpenClPointCloudBackend() override {
        if(queue != nullptr) clFinish(queue);
        unmapOutput();
        zeroCopyInputs.clear();
        stagedDepth.release();
        stagedColor.release();
        rays.release();
        transform.release();
        output.release();
        outputMemory.reset();
        if(coloredKernel != nullptr) clReleaseKernel(coloredKernel);
        if(kernel != nullptr) clReleaseKernel(kernel);
        if(queue != nullptr) clReleaseCommandQueue(queue);
    }

    const Point3f* computeDense(const Geometry& geometry, const std::uint8_t* depthData, const std::shared_ptr<Memory>& depthMemory) override {
        const std::size_t count = static_cast<std::size_t>(geometry.width) * geometry.height;
        if(count == 0) return nullptr;
        unmapOutput();
        prepareConstants(geometry, count);
        const auto depth = bindInput(depthData, depthMemory, count * sizeof(std::uint16_t), stagedDepth);
        const cl_int depthOffsetElements = depth.offsetBytes / static_cast<cl_int>(sizeof(std::uint16_t));
        ensureOutput(count * sizeof(Point3f));

        cl_uint arg = 0;
        setArg(kernel, arg++, depth.buffer);
        setArg(kernel, arg++, depthOffsetElements);
        setArg(kernel, arg++, rays.buffer);
        setArg(kernel, arg++, output.buffer);
        setArg(kernel, arg++, geometry.depthScale);
        setArg(kernel, arg++, static_cast<cl_int>(count));
        setArg(kernel, arg++, static_cast<cl_int>(geometry.hasTransform ? 1 : 0));
        setArg(kernel, arg++, transform.buffer);
        return static_cast<const Point3f*>(run(kernel, count, count * sizeof(Point3f)));
    }

    const Point3fRGBA* computeDenseColored(const Geometry& geometry,
                                           const std::uint8_t* depthData,
                                           const std::shared_ptr<Memory>& depthMemory,
                                           const std::uint8_t* colorData,
                                           const std::shared_ptr<Memory>& colorMemory) override {
        static_assert(sizeof(Point3fRGBA) == 16, "Point3fRGBA layout expected to be xyz floats + rgba bytes");
        const std::size_t count = static_cast<std::size_t>(geometry.width) * geometry.height;
        if(count == 0) return nullptr;
        unmapOutput();
        prepareConstants(geometry, count);
        const auto depth = bindInput(depthData, depthMemory, count * sizeof(std::uint16_t), stagedDepth);
        const auto color = bindInput(colorData, colorMemory, count * 3, stagedColor);
        const cl_int depthOffsetElements = depth.offsetBytes / static_cast<cl_int>(sizeof(std::uint16_t));
        ensureOutput(count * sizeof(Point3fRGBA));

        cl_uint arg = 0;
        setArg(coloredKernel, arg++, depth.buffer);
        setArg(coloredKernel, arg++, depthOffsetElements);
        setArg(coloredKernel, arg++, color.buffer);
        setArg(coloredKernel, arg++, color.offsetBytes);
        setArg(coloredKernel, arg++, rays.buffer);
        setArg(coloredKernel, arg++, output.buffer);
        setArg(coloredKernel, arg++, geometry.depthScale);
        setArg(coloredKernel, arg++, static_cast<cl_int>(count));
        setArg(coloredKernel, arg++, static_cast<cl_int>(geometry.hasTransform ? 1 : 0));
        setArg(coloredKernel, arg++, transform.buffer);
        return static_cast<const Point3fRGBA*>(run(coloredKernel, count, count * sizeof(Point3fRGBA)));
    }

   private:
    template <typename T>
    static void setArg(cl_kernel k, cl_uint index, const T& value) {
        utility::checkStatus(clSetKernelArg(k, index, sizeof(T), &value), "clSetKernelArg");
    }

    void prepareConstants(const Geometry& geometry, std::size_t count) {
        const std::size_t rayBytes = count * 2 * sizeof(float);  // Point2f is more than two floats: the table is packed into float2 pairs
        if(!rays.matches(rayBytes, CL_MEM_READ_ONLY, nullptr) || raysVersion != geometry.raysVersion) {
            if(!rays.matches(rayBytes, CL_MEM_READ_ONLY, nullptr)) {
                rays.release();
                utility::validateBufferSize(shared->device, rayBytes, "clCreateBuffer(rays)");
                cl_int status = CL_SUCCESS;
                rays.buffer = clCreateBuffer(shared->context, CL_MEM_READ_ONLY, rayBytes, nullptr, &status);
                utility::checkStatus(status, "clCreateBuffer(rays)");
                rays.sizeBytes = rayBytes;
                rays.flags = CL_MEM_READ_ONLY;
            }
            if(geometry.rays == nullptr) throw utility::ClExecutionError("PointCloud GPU backend: ray table missing");
            packedRays.resize(count * 2);
            for(std::size_t i = 0; i < count; ++i) {
                packedRays[2 * i] = geometry.rays[i].x;
                packedRays[2 * i + 1] = geometry.rays[i].y;
            }
            utility::checkStatus(clEnqueueWriteBuffer(queue, rays.buffer, CL_TRUE, 0, rayBytes, packedRays.data(), 0, nullptr, nullptr),
                                 "clEnqueueWriteBuffer(rays)");
            raysVersion = geometry.raysVersion;
        }

        std::array<float, 12> flat{};
        if(geometry.hasTransform) {
            for(std::size_t r = 0; r < 3; ++r) {
                for(std::size_t c = 0; c < 4; ++c) flat[r * 4 + c] = geometry.transform[r][c];
            }
        }
        if(transform.buffer == nullptr) {
            cl_int status = CL_SUCCESS;
            transform.buffer = clCreateBuffer(shared->context, CL_MEM_READ_ONLY, sizeof(flat), nullptr, &status);
            utility::checkStatus(status, "clCreateBuffer(transform)");
            transform.sizeBytes = sizeof(flat);
            transform.flags = CL_MEM_READ_ONLY;
            transformValid = false;
        }
        if(!transformValid || flat != currentTransform) {
            utility::checkStatus(clEnqueueWriteBuffer(queue, transform.buffer, CL_TRUE, 0, sizeof(flat), flat.data(), 0, nullptr, nullptr),
                                 "clEnqueueWriteBuffer(transform)");
            currentTransform = flat;
            transformValid = true;
        }
    }

    void ensureOutput(std::size_t bytes) {  // Own cached dma-buf: CL_MEM_ALLOC_HOST_PTR maps uncached here, too slow to read 12 MB
        if(output.buffer != nullptr && output.sizeBytes == bytes) return;
        unmapOutput();
        output.release();
        outputMemory.reset();
        utility::validateBufferSize(shared->device, bytes, "clCreateBuffer(points)");
        cl_int status = CL_SUCCESS;
#if POINTCLOUD_GPU_ZERO_COPY
        if(shared->dmabufHostPtr || shared->ionHostPtr) {
            try {
                auto memory = std::make_shared<DmaHeapMemory>(bytes);
                if(auto info = utility::getExternalMemoryInfo(memory)) {
                    info->hostCachePolicy = CL_MEM_HOST_WRITEBACK_QCOM;  // IO-coherent avoids a cache invalidate pass over 12 MB per frame
    #if defined(CL_MEM_HOST_IOCOHERENT_QCOM)
                    if(shared->ioCoherentHostPtr) info->hostCachePolicy = CL_MEM_HOST_IOCOHERENT_QCOM;
    #endif
                    outputIoCoherent = info->hostCachePolicy != CL_MEM_HOST_WRITEBACK_QCOM;
                    const cl_mem_flags flags = CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM;
                    cl_mem buffer = createImportedBuffer(flags, *info, memory->getData().size(), status);
                    if(buffer != nullptr && status == CL_SUCCESS) {
                        output.buffer = buffer;
                        output.sizeBytes = bytes;
                        output.flags = flags;
                        output.hostPtr = info->hostPtr;
                        output.dmaBufFd = info->dmaBufFd;
                        outputMemory = std::move(memory);
                        return;
                    }
                }
            } catch(const std::exception& ex) {
                if(logger && !outputImportWarned)
                    logger->warn("PointCloud GPU backend: dma-buf output not available ({}), using a driver allocated buffer", ex.what());
                outputImportWarned = true;
            }
            if(logger && !outputImportWarned)
                logger->warn("PointCloud GPU backend: dma-buf output import failed with OpenCL error {}, using a driver allocated buffer", status);
            outputImportWarned = true;
        }
#endif
        output.buffer = clCreateBuffer(shared->context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, bytes, nullptr, &status);
        utility::checkStatus(status, "clCreateBuffer(points)");
        output.sizeBytes = bytes;
        output.flags = CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR;
    }

    InputBinding bindInput(const std::uint8_t* data, const std::shared_ptr<Memory>& memory, std::size_t requiredBytes, ReusableBuffer& staged) {
#if POINTCLOUD_GPU_ZERO_COPY
        if(memory && (shared->dmabufHostPtr || shared->ionHostPtr)) {
            if(auto offsetMemory = std::dynamic_pointer_cast<OffsetMemory>(memory)) {
                if(auto info = utility::getExternalMemoryInfo(offsetMemory)) {
                    const std::size_t totalBytes = offsetMemory->getData().size();
                    const std::size_t offset = offsetMemory->getOffset();
                    if(offset + requiredBytes <= totalBytes && offset <= static_cast<std::size_t>(std::numeric_limits<cl_int>::max()) && offset % 4 == 0) {
                        if(auto* imported = importZeroCopy(*info, totalBytes)) {
                            return {imported->buffer, static_cast<cl_int>(offset)};
                        }
                    }
                }
            }
        }
#endif
        if(!staged.matches(requiredBytes, CL_MEM_READ_ONLY, nullptr)) {
            staged.release();
            utility::validateBufferSize(shared->device, requiredBytes, "clCreateBuffer(input)");
            cl_int status = CL_SUCCESS;
            staged.buffer = clCreateBuffer(shared->context, CL_MEM_READ_ONLY, requiredBytes, nullptr, &status);
            utility::checkStatus(status, "clCreateBuffer(input)");
            staged.sizeBytes = requiredBytes;
            staged.flags = CL_MEM_READ_ONLY;
        }
        cl_event uploadEvent = nullptr;
        utility::checkStatus(clEnqueueWriteBuffer(queue, staged.buffer, CL_FALSE, 0, requiredBytes, data, 0, nullptr, &uploadEvent),
                             "clEnqueueWriteBuffer(input)");
        pendingUploads.push_back(uploadEvent);
        return {staged.buffer, 0};
    }

#if POINTCLOUD_GPU_ZERO_COPY
    cl_mem createImportedBuffer(cl_mem_flags flags, const utility::ExternalMemoryInfo& info, std::size_t totalBytes, cl_int& status) {
        status = CL_SUCCESS;
        cl_mem buffer = nullptr;
        if(shared->dmabufHostPtr) {
    #if defined(CL_MEM_DMABUF_HOST_PTR_QCOM)
            cl_mem_dmabuf_host_ptr hostPtr = {};
            hostPtr.ext_host_ptr.allocation_type = CL_MEM_DMABUF_HOST_PTR_QCOM;
            hostPtr.ext_host_ptr.host_cache_policy = info.hostCachePolicy;
            hostPtr.dmabuf_filedesc = info.dmaBufFd;
            hostPtr.dmabuf_hostptr = info.hostPtr;
            buffer = clCreateBuffer(shared->context, flags, totalBytes, &hostPtr, &status);
    #endif
        } else if(shared->ionHostPtr) {
    #if defined(CL_MEM_ION_HOST_PTR_QCOM)
            cl_mem_ion_host_ptr hostPtr = {};
            hostPtr.ext_host_ptr.allocation_type = CL_MEM_ION_HOST_PTR_QCOM;
            hostPtr.ext_host_ptr.host_cache_policy = info.hostCachePolicy;
            hostPtr.ion_filedesc = info.dmaBufFd;
            hostPtr.ion_hostptr = info.hostPtr;
            buffer = clCreateBuffer(shared->context, flags, totalBytes, &hostPtr, &status);
    #endif
        } else {
            status = CL_INVALID_OPERATION;
        }
        if(status != CL_SUCCESS && buffer != nullptr) {
            clReleaseMemObject(buffer);
            buffer = nullptr;
        }
        return buffer;
    }

    ReusableBuffer* importZeroCopy(utility::ExternalMemoryInfo info, std::size_t totalBytes) {
        const cl_mem_flags flags = CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM;
        for(auto& cached : zeroCopyInputs) {
            if(cached->matches(totalBytes, flags, info.hostPtr) && cached->dmaBufFd == info.dmaBufFd) return cached.get();
        }
        if(zeroCopyInputs.size() >= 16) zeroCopyInputs.clear();  // Frame pools rotate over a handful of buffers
    #if defined(CL_MEM_HOST_IOCOHERENT_QCOM)
        if(shared->ioCoherentHostPtr) info.hostCachePolicy = CL_MEM_HOST_IOCOHERENT_QCOM;
    #endif
        cl_int status = CL_SUCCESS;
        cl_mem buffer = createImportedBuffer(flags, info, totalBytes, status);
        if(status != CL_SUCCESS || buffer == nullptr) {
            if(logger && !zeroCopyWarned) {
                logger->warn("PointCloud GPU backend: zero-copy import failed with OpenCL error {}; using staged copies", status);
                zeroCopyWarned = true;
            }
            return nullptr;
        }
        auto cached = std::make_unique<ReusableBuffer>();
        cached->buffer = buffer;
        cached->sizeBytes = totalBytes;
        cached->flags = flags;
        cached->hostPtr = info.hostPtr;
        cached->dmaBufFd = info.dmaBufFd;
        zeroCopyInputs.push_back(std::move(cached));
        return zeroCopyInputs.back().get();
    }
#endif

    const void* run(cl_kernel k, std::size_t count, std::size_t bytes) {
        const std::size_t global = count;
        cl_event kernelEvent = nullptr;
        cl_event mapEvent = nullptr;
        utility::checkStatus(clEnqueueNDRangeKernel(queue, k, 1, nullptr, &global, nullptr, 0, nullptr, &kernelEvent), "clEnqueueNDRangeKernel");
        cl_int status = CL_SUCCESS;
        mappedOutput = clEnqueueMapBuffer(queue, output.buffer, CL_TRUE, CL_MAP_READ, 0, bytes, 0, nullptr, &mapEvent, &status);
        utility::checkStatus(status, "clEnqueueMapBuffer(points)");
        if(outputMemory) {
            if(!outputIoCoherent) {
                outputMemory->syncStartCpuAccess(DmaHeapMemory::CpuSyncAccess::Read);
                cpuAccessActive = true;
            }
            mappedOutput = outputMemory->getData().data();
        }
        recordProfile(kernelEvent, mapEvent, count);
        return mappedOutput;
    }

    void unmapOutput() {
        if(cpuAccessActive && outputMemory) {
            outputMemory->syncEndCpuAccess(DmaHeapMemory::CpuSyncAccess::Read);
        }
        cpuAccessActive = false;
        if(mappedOutput != nullptr && output.buffer != nullptr) {
            void* mapped = outputMemory ? static_cast<void*>(outputMemory->getData().data()) : mappedOutput;  // The map returned the import's host pointer
            clEnqueueUnmapMemObject(queue, output.buffer, mapped, 0, nullptr, nullptr);
        }
        mappedOutput = nullptr;
    }

    static double eventMilliseconds(cl_event event) {
        cl_ulong start = 0, end = 0;
        if(clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_START, sizeof(start), &start, nullptr) != CL_SUCCESS) return 0.0;
        if(clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_END, sizeof(end), &end, nullptr) != CL_SUCCESS) return 0.0;
        return static_cast<double>(end - start) * 1e-6;
    }

    void recordProfile(cl_event kernelEvent, cl_event mapEvent, std::size_t count) {
        for(cl_event upload : pendingUploads) {
            profileUploadMs += eventMilliseconds(upload);
            clReleaseEvent(upload);
        }
        profileZeroCopyFrames += pendingUploads.empty() ? 1 : 0;
        pendingUploads.clear();
        profileKernelMs += eventMilliseconds(kernelEvent);
        profileMapMs += eventMilliseconds(mapEvent);
        clReleaseEvent(kernelEvent);
        clReleaseEvent(mapEvent);
        if(++profileFrames >= PROFILE_INTERVAL) {
            if(logger) {
                logger->debug("PointCloud GPU: {} points, per frame upload {:.2f} ms, kernel {:.2f} ms, map {:.2f} ms ({} of {} frames imported zero-copy)",
                              count,
                              profileUploadMs / profileFrames,
                              profileKernelMs / profileFrames,
                              profileMapMs / profileFrames,
                              profileZeroCopyFrames,
                              profileFrames);
            }
            profileFrames = 0;
            profileZeroCopyFrames = 0;
            profileUploadMs = profileKernelMs = profileMapMs = 0.0;
        }
    }

    void selfTest() {
        constexpr unsigned W = 4, H = 2;
        std::array<std::uint16_t, W * H> depth = {0, 1000, 2000, 3000, 500, 0, 1500, 2500};
        std::array<Point2f, W * H> rayTable{};
        for(unsigned i = 0; i < W * H; ++i) rayTable[i] = {0.1f * i - 0.3f, 0.05f * i};
        Geometry g;
        g.width = W;
        g.height = H;
        g.depthScale = 0.001f;
        g.rays = rayTable.data();
        g.raysVersion = 1;
        g.hasTransform = true;
        g.transform = {{{0, -1, 0, 0.1f}, {1, 0, 0, 0.2f}, {0, 0, 1, 0.3f}, {0, 0, 0, 1}}};
        std::array<Point3f, W * H> points{};
        const auto* mappedPoints = computeDense(g, reinterpret_cast<const std::uint8_t*>(depth.data()), nullptr);
        if(mappedPoints == nullptr) throw utility::ClExecutionError("PointCloud GPU backend: self-test returned no points");
        std::copy(mappedPoints, mappedPoints + W * H, points.begin());
        std::array<std::uint8_t, W * H * 3> color{};
        for(std::size_t i = 0; i < color.size(); ++i) color[i] = static_cast<std::uint8_t>(i * 7);
        std::array<Point3fRGBA, W * H> colored{};
        const auto* mappedColored = computeDenseColored(g, reinterpret_cast<const std::uint8_t*>(depth.data()), nullptr, color.data(), nullptr);
        if(mappedColored == nullptr) throw utility::ClExecutionError("PointCloud GPU backend: self-test returned no colored points");
        std::copy(mappedColored, mappedColored + W * H, colored.begin());
        unmapOutput();
        for(unsigned i = 0; i < W * H; ++i) {
            const float z0 = static_cast<float>(depth[i]) * g.depthScale;
            float ex = 0.f, ey = 0.f, ez = z0;
            if(z0 > 0.f) {
                const float x = rayTable[i].x * z0, y = rayTable[i].y * z0;
                ex = -y + 0.1f;
                ey = x + 0.2f;
                ez = z0 + 0.3f;
            }
            const bool ok = std::fabs(points[i].x - ex) < 1e-4f && std::fabs(points[i].y - ey) < 1e-4f && std::fabs(points[i].z - ez) < 1e-4f
                            && std::fabs(colored[i].x - ex) < 1e-4f && std::fabs(colored[i].z - ez) < 1e-4f && colored[i].r == color[3 * i]
                            && colored[i].g == color[3 * i + 1] && colored[i].b == color[3 * i + 2] && colored[i].a == 255;
            if(!ok) throw utility::ClExecutionError("PointCloud GPU backend: startup self-test produced wrong points");
        }
        raysVersion = 0;  // So that the self-test geometry is not taken for the first real frame
        rays.release();
        profileFrames = 0;
        profileZeroCopyFrames = 0;
        profileUploadMs = profileKernelMs = profileMapMs = 0.0;
    }

    std::shared_ptr<SharedContext> shared;
    std::shared_ptr<spdlog::logger> logger;
    cl_command_queue queue = nullptr;
    cl_kernel kernel = nullptr;
    cl_kernel coloredKernel = nullptr;
    ReusableBuffer rays, transform, output, stagedDepth, stagedColor;
    std::vector<std::unique_ptr<ReusableBuffer>> zeroCopyInputs;
    void* mappedOutput = nullptr;
    std::shared_ptr<DmaHeapMemory> outputMemory;  // set when the output is a zero-copy dma-buf
    bool cpuAccessActive = false;
    bool outputIoCoherent = false;
    bool outputImportWarned = false;
    std::vector<cl_event> pendingUploads;
    static constexpr unsigned PROFILE_INTERVAL = 30;
    unsigned profileFrames = 0;
    unsigned profileZeroCopyFrames = 0;
    double profileUploadMs = 0.0, profileKernelMs = 0.0, profileMapMs = 0.0;
    std::uint64_t raysVersion = 0;
    std::vector<float> packedRays;
    std::array<float, 12> currentTransform{};
    bool transformValid = false;
    bool zeroCopyWarned = false;
};

}  // namespace

std::shared_ptr<node::internal::PointCloudGpuBackend> createOpenClPointCloudBackend(std::uint32_t gpuDevice, std::shared_ptr<spdlog::logger> logger) {
    try {
        return std::make_shared<OpenClPointCloudBackend>(gpuDevice, logger);
    } catch(const std::exception& ex) {
        if(logger) logger->warn("PointCloud GPU backend unavailable: {}", ex.what());
        return nullptr;
    }
}

}  // namespace gate
}  // namespace dai
