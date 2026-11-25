#include "ImageManipGPUMat.hpp"

#include <CL/cl.h>
#include <CL/cl_ext.h>
#if defined(__has_include)
    #if __has_include(<CL/cl_ext_qcom.h>)
        #include <CL/cl_ext_qcom.h>
        #define DEPTHAI_HAVE_CL_EXT_QCOM 1
    #endif
#endif

#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "depthai/utility/matrixOps.hpp"

namespace dai::gate {
namespace {

// TODO different interpolation methods, determine size at which GPU runs better than CPU, determine size at which
// zero-copy overhead is not worth it, implement kernel path that uses opencl images

const char* kWarpKernelSource = R"CLC(
inline float sample_uchar_constant_border(__global const uchar* srcBytes,
                                          __constant const ushort* borderValue,
                                          int srcWidth,
                                          int srcHeight,
                                          int srcStrideBytes,
                                          int pixelStrideBytes,
                                          int x,
                                          int y,
                                          int channel) {
    if(x < 0 || x >= srcWidth || y < 0 || y >= srcHeight) {
        return (float)borderValue[channel];
    }

    return (float)srcBytes[y * srcStrideBytes + x * pixelStrideBytes + channel];
}

inline float sample_ushort_constant_border(__global const ushort* srcShorts,
                                           __constant const ushort* borderValue,
                                           int srcWidth,
                                           int srcHeight,
                                           int srcStrideShorts,
                                           int channels,
                                           int x,
                                           int y,
                                           int channel) {
    if(x < 0 || x >= srcWidth || y < 0 || y >= srcHeight) {
        return (float)borderValue[channel];
    }

    return (float)srcShorts[y * srcStrideShorts + x * channels + channel];
}

__kernel void warp_perspective_linear(
        __global const uchar* srcBytes,
        __global uchar* dstBytes,
        __constant const float* inverseMatrix,
        __constant const ushort* borderValue,
        int srcOffsetBytes,
        int dstOffsetBytes,
        int srcWidth,
        int srcHeight,
        int srcStrideBytes,
        int dstWidth,
        int dstHeight,
        int dstStrideBytes,
        int channels,
        int bytesPerChannel) {
    const int x = get_global_id(0);
    const int y = get_global_id(1);
    if(x >= dstWidth || y >= dstHeight) {
        return;
    }

    const float xf = (float)x;
    const float yf = (float)y;

    const float h00 = inverseMatrix[0];
    const float h01 = inverseMatrix[1];
    const float h02 = inverseMatrix[2];
    const float h10 = inverseMatrix[3];
    const float h11 = inverseMatrix[4];
    const float h12 = inverseMatrix[5];
    const float h20 = inverseMatrix[6];
    const float h21 = inverseMatrix[7];
    const float h22 = inverseMatrix[8];
    __global const uchar* srcBaseBytes = srcBytes + srcOffsetBytes;
    __global uchar* dstBaseBytes = dstBytes + dstOffsetBytes;

    const float denom = h20 * xf + h21 * yf + h22;
    const int dstPixelStride = channels * bytesPerChannel;
    __global uchar* dstPixel = dstBaseBytes + y * dstStrideBytes + x * dstPixelStride;

    if(fabs(denom) < 1.0e-8f) {
        for(int c = 0; c < channels; ++c) {
            if(bytesPerChannel == 1) {
                dstPixel[c] = convert_uchar_sat(borderValue[c]);
            } else {
                ((__global ushort*)dstPixel)[c] = borderValue[c];
            }
        }
        return;
    }

    const float srcX = (h00 * xf + h01 * yf + h02) / denom;
    const float srcY = (h10 * xf + h11 * yf + h12) / denom;

    if(srcX <= -1.0f || srcY <= -1.0f || srcX >= (float)srcWidth || srcY >= (float)srcHeight) {
        for(int c = 0; c < channels; ++c) {
            if(bytesPerChannel == 1) {
                dstPixel[c] = convert_uchar_sat(borderValue[c]);
            } else {
                ((__global ushort*)dstPixel)[c] = borderValue[c];
            }
        }
        return;
    }

    const int x0 = (int)floor(srcX);
    const int y0 = (int)floor(srcY);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float tx = srcX - (float)x0;
    const float ty = srcY - (float)y0;
    const float oneMinusTx = 1.0f - tx;
    const float oneMinusTy = 1.0f - ty;
    const float w00 = oneMinusTx * oneMinusTy;
    const float w10 = tx * oneMinusTy;
    const float w01 = oneMinusTx * ty;
    const float w11 = tx * ty;

    if(bytesPerChannel == 1) {
        for(int c = 0; c < channels; ++c) {
            const float c00 = sample_uchar_constant_border(
                srcBaseBytes, borderValue, srcWidth, srcHeight, srcStrideBytes, dstPixelStride, x0, y0, c);
            const float c10 = sample_uchar_constant_border(
                srcBaseBytes, borderValue, srcWidth, srcHeight, srcStrideBytes, dstPixelStride, x1, y0, c);
            const float c01 = sample_uchar_constant_border(
                srcBaseBytes, borderValue, srcWidth, srcHeight, srcStrideBytes, dstPixelStride, x0, y1, c);
            const float c11 = sample_uchar_constant_border(
                srcBaseBytes, borderValue, srcWidth, srcHeight, srcStrideBytes, dstPixelStride, x1, y1, c);
            const float value = c00 * w00 + c10 * w10 + c01 * w01 + c11 * w11;
            dstPixel[c] = convert_uchar_sat_rte(value);
        }
    } else {
        __global const ushort* srcShorts = (__global const ushort*)srcBaseBytes;
        const int srcStrideShorts = srcStrideBytes / 2;
        __global ushort* dstShorts = (__global ushort*)dstPixel;

        for(int c = 0; c < channels; ++c) {
            const float c00 =
                sample_ushort_constant_border(srcShorts, borderValue, srcWidth, srcHeight, srcStrideShorts, channels, x0, y0, c);
            const float c10 =
                sample_ushort_constant_border(srcShorts, borderValue, srcWidth, srcHeight, srcStrideShorts, channels, x1, y0, c);
            const float c01 =
                sample_ushort_constant_border(srcShorts, borderValue, srcWidth, srcHeight, srcStrideShorts, channels, x0, y1, c);
            const float c11 =
                sample_ushort_constant_border(srcShorts, borderValue, srcWidth, srcHeight, srcStrideShorts, channels, x1, y1, c);
            const float value = c00 * w00 + c10 * w10 + c01 * w01 + c11 * w11;
            dstShorts[c] = convert_ushort_sat_rte(value);
        }
    }
}
)CLC";

struct WarpKernelArgs {
    cl_mem srcBuffer = nullptr;
    cl_mem dstBuffer = nullptr;
    cl_mem inverseMatrixBuffer = nullptr;
    cl_mem borderValueBuffer = nullptr;
    cl_int srcOffsetBytes = 0;
    cl_int dstOffsetBytes = 0;
    cl_int srcWidth = 0;
    cl_int srcHeight = 0;
    cl_int srcStrideBytes = 0;
    cl_int dstWidth = 0;
    cl_int dstHeight = 0;
    cl_int dstStrideBytes = 0;
    cl_int channels = 0;
    cl_int bytesPerChannel = 0;
};

template <uint32_t N, uint32_t M>
std::array<float, (size_t)(N * M)> flatten(const std::array<std::array<float, M>, N>& matrix) {
    std::array<float, (size_t)(N * M)> ret{};
    for(auto i = 0u; i < N; ++i)
        for(auto j = 0u; j < M; ++j) ret[(i * M) + j] = matrix[i][j];
    return ret;
}

void validateConfig(const MatConfig& config) {
    if(config.srcWidth == 0 || config.srcHeight == 0 || config.dstWidth == 0 || config.dstHeight == 0) {
        throw std::invalid_argument("WarpGPU dimensions must be non-zero");
    }
    if(config.srcStrideBytes == 0 || config.dstStrideBytes == 0) {
        throw std::invalid_argument("WarpGPU strides must be non-zero");
    }
    if(config.channels == 0 || config.channels > 4) {
        throw std::invalid_argument("WarpGPU supports between 1 and 4 channels");
    }
    if(config.bytesPerChannel != 1 && config.bytesPerChannel != 2) {
        throw std::invalid_argument("WarpGPU supports 8-bit and 16-bit channels only");
    }
}

WarpKernelArgs makeWarpKernelArgs(cl_mem srcBuffer,
                                  cl_mem dstBuffer,
                                  cl_mem inverseMatrixBuffer,
                                  cl_mem borderValueBuffer,
                                  cl_int srcOffsetBytes,
                                  cl_int dstOffsetBytes,
                                  cl_int srcWidth,
                                  cl_int srcHeight,
                                  cl_int srcStrideBytes,
                                  cl_int dstWidth,
                                  cl_int dstHeight,
                                  cl_int dstStrideBytes,
                                  cl_int channels,
                                  cl_int bytesPerChannel) {
    WarpKernelArgs args;
    args.srcBuffer = srcBuffer;
    args.dstBuffer = dstBuffer;
    args.inverseMatrixBuffer = inverseMatrixBuffer;
    args.borderValueBuffer = borderValueBuffer;
    args.srcOffsetBytes = srcOffsetBytes;
    args.dstOffsetBytes = dstOffsetBytes;
    args.srcWidth = srcWidth;
    args.srcHeight = srcHeight;
    args.srcStrideBytes = srcStrideBytes;
    args.dstWidth = dstWidth;
    args.dstHeight = dstHeight;
    args.dstStrideBytes = dstStrideBytes;
    args.channels = channels;
    args.bytesPerChannel = bytesPerChannel;
    return args;
}

WarpKernelArgs makeWarpKernelArgs(cl_mem srcBuffer,
                                  cl_mem dstBuffer,
                                  cl_mem inverseMatrixBuffer,
                                  cl_mem borderValueBuffer,
                                  cl_int srcOffsetBytes,
                                  cl_int dstOffsetBytes,
                                  const MatConfig& config) {
    return makeWarpKernelArgs(srcBuffer,
                              dstBuffer,
                              inverseMatrixBuffer,
                              borderValueBuffer,
                              srcOffsetBytes,
                              dstOffsetBytes,
                              static_cast<cl_int>(config.srcWidth),
                              static_cast<cl_int>(config.srcHeight),
                              static_cast<cl_int>(config.srcStrideBytes),
                              static_cast<cl_int>(config.dstWidth),
                              static_cast<cl_int>(config.dstHeight),
                              static_cast<cl_int>(config.dstStrideBytes),
                              static_cast<cl_int>(config.channels),
                              static_cast<cl_int>(config.bytesPerChannel));
}

void setWarpKernelArgs(cl_kernel kernel, const WarpKernelArgs& args, const char* contextLabel) {
    auto setArg = [&](cl_uint index, std::size_t size, const void* value, const char* name) {
        std::ostringstream what;
        what << "clSetKernelArg(";
        if(contextLabel != nullptr && contextLabel[0] != '\0') {
            what << contextLabel << ':';
        }
        what << name << ')';
        const std::string whatString = what.str();
        utility::checkStatus(clSetKernelArg(kernel, index, size, value), whatString.c_str());
    };

    setArg(0, sizeof(cl_mem), &args.srcBuffer, "src");
    setArg(1, sizeof(cl_mem), &args.dstBuffer, "dst");
    setArg(2, sizeof(cl_mem), &args.inverseMatrixBuffer, "inverseMatrix");
    setArg(3, sizeof(cl_mem), &args.borderValueBuffer, "borderValue");
    setArg(4, sizeof(cl_int), &args.srcOffsetBytes, "srcOffsetBytes");
    setArg(5, sizeof(cl_int), &args.dstOffsetBytes, "dstOffsetBytes");
    setArg(6, sizeof(cl_int), &args.srcWidth, "srcWidth");
    setArg(7, sizeof(cl_int), &args.srcHeight, "srcHeight");
    setArg(8, sizeof(cl_int), &args.srcStrideBytes, "srcStrideBytes");
    setArg(9, sizeof(cl_int), &args.dstWidth, "dstWidth");
    setArg(10, sizeof(cl_int), &args.dstHeight, "dstHeight");
    setArg(11, sizeof(cl_int), &args.dstStrideBytes, "dstStrideBytes");
    setArg(12, sizeof(cl_int), &args.channels, "channels");
    setArg(13, sizeof(cl_int), &args.bytesPerChannel, "bytesPerChannel");
}

bool runStartupCorrectnessProbe(cl_context context, cl_device_id device, cl_command_queue queue, cl_kernel kernel) {
    constexpr cl_int width = 5;
    constexpr cl_int height = 4;
    constexpr cl_int strideBytes = width;
    constexpr cl_int channels = 1;
    constexpr cl_int bytesPerChannel = 1;
    constexpr size_t bufferSize = static_cast<size_t>(width * height);
    const size_t globalWorkSize[2] = {static_cast<size_t>(width), static_cast<size_t>(height)};
    const std::array<float, 9> inverseMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const std::array<std::uint16_t, 4> borderValue = {0, 0, 0, 0};
    const std::array<std::uint8_t, bufferSize> srcData = {1, 17, 33, 49, 65, 81, 97, 113, 129, 145, 161, 177, 193, 209, 225, 241, 13, 29, 45, 61};
    std::array<std::uint8_t, bufferSize> dstData{};

    cl_mem srcBuffer = nullptr;
    cl_mem dstBuffer = nullptr;
    cl_mem inverseMatrixBuffer = nullptr;
    cl_mem borderValueBuffer = nullptr;
    cl_int status = CL_SUCCESS;

    auto releaseBuffer = [](cl_mem& buffer) {
        if(buffer != nullptr) {
            clReleaseMemObject(buffer);
            buffer = nullptr;
        }
    };

    try {
        utility::validateBufferSize(device, srcData.size() * sizeof(srcData[0]), "clCreateBuffer(startupProbeSrc)");
        srcBuffer = clCreateBuffer(
            context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, srcData.size() * sizeof(srcData[0]), const_cast<std::uint8_t*>(srcData.data()), &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeSrc)");
        utility::validateBufferSize(device, dstData.size() * sizeof(dstData[0]), "clCreateBuffer(startupProbeDst)");
        dstBuffer = clCreateBuffer(context, CL_MEM_WRITE_ONLY, dstData.size() * sizeof(dstData[0]), nullptr, &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeDst)");
        utility::validateBufferSize(device,
                                    inverseMatrix.size() * sizeof(inverseMatrix[0]),
                                    "clCreateBuffer(startupProbeInverseMatrix)");
        inverseMatrixBuffer = clCreateBuffer(context,
                                             CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                             inverseMatrix.size() * sizeof(inverseMatrix[0]),
                                             const_cast<float*>(inverseMatrix.data()),
                                             &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeInverseMatrix)");
        utility::validateBufferSize(device, borderValue.size() * sizeof(borderValue[0]), "clCreateBuffer(startupProbeBorderValue)");
        borderValueBuffer = clCreateBuffer(context,
                                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                           borderValue.size() * sizeof(borderValue[0]),
                                           const_cast<std::uint16_t*>(borderValue.data()),
                                           &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeBorderValue)");

        const WarpKernelArgs kernelArgs = makeWarpKernelArgs(srcBuffer,
                                                             dstBuffer,
                                                             inverseMatrixBuffer,
                                                             borderValueBuffer,
                                                             0,
                                                             0,
                                                             width,
                                                             height,
                                                             strideBytes,
                                                             width,
                                                             height,
                                                             strideBytes,
                                                             channels,
                                                             bytesPerChannel);
        setWarpKernelArgs(kernel, kernelArgs, "startupProbe");

        utility::checkStatus(clEnqueueNDRangeKernel(queue, kernel, 2, nullptr, globalWorkSize, nullptr, 0, nullptr, nullptr), "clEnqueueNDRangeKernel(startupProbe)");
        utility::checkStatus(clEnqueueReadBuffer(queue, dstBuffer, CL_TRUE, 0, dstData.size() * sizeof(dstData[0]), dstData.data(), 0, nullptr, nullptr),
                    "clEnqueueReadBuffer(startupProbe)");
    } catch(...) {
        releaseBuffer(borderValueBuffer);
        releaseBuffer(inverseMatrixBuffer);
        releaseBuffer(dstBuffer);
        releaseBuffer(srcBuffer);
        throw;
    }

    releaseBuffer(borderValueBuffer);
    releaseBuffer(inverseMatrixBuffer);
    releaseBuffer(dstBuffer);
    releaseBuffer(srcBuffer);

    return dstData == srcData;
}

}  // namespace

WarpGPUMat::~WarpGPUMat() {
    releaseOpenClObjects(false);
}

bool WarpGPUMat::init(int numPlanes) {
    if(initialized) {
        if(static_cast<std::size_t>(numPlanes) == planeData.size()) {
            return true;
        }

        releaseOpenClObjects(true);
        planeData.clear();
        planeData.resize(numPlanes);
        return true;
    }

    releaseOpenClObjects(false);
    cl_int status = CL_SUCCESS;

    try {
        utility::checkStatus(clGetPlatformIDs(1, &platform, nullptr), "clGetPlatformIDs");
        status = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, nullptr);
        if(status != CL_SUCCESS) {
            return false;
        }

        const std::string deviceExtensions = utility::getDeviceExtensions(device);
        qcomIoCoherentHostPtrSupported = utility::hasExtensionToken(deviceExtensions, "cl_qcom_ext_host_ptr_iocoherent");
        const bool qcomDmabufHostPtrAdvertised = utility::hasExtensionToken(deviceExtensions, "cl_qcom_dmabuf_host_ptr");
        qcomDmabufHostPtrSupported =
#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && defined(CL_MEM_DMABUF_HOST_PTR_QCOM)
            utility::hasExtensionToken(deviceExtensions, "cl_qcom_ext_host_ptr") && qcomDmabufHostPtrAdvertised;
#else
            false;
#endif
        qcomIonHostPtrSupported =
#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && defined(CL_MEM_ION_HOST_PTR_QCOM)
            utility::hasExtensionToken(deviceExtensions, "cl_qcom_ext_host_ptr") && utility::hasExtensionToken(deviceExtensions, "cl_qcom_ion_host_ptr");
#else
            false;
#endif

        context = clCreateContext(nullptr, 1, &device, nullptr, nullptr, &status);
        utility::checkStatus(status, "clCreateContext");

#if CL_TARGET_OPENCL_VERSION >= 200
        queue = clCreateCommandQueueWithProperties(context, device, nullptr, &status);
#else
        queue = clCreateCommandQueue(context, device, 0, &status);
#endif
        utility::checkStatus(status, "clCreateCommandQueue");

        program = clCreateProgramWithSource(context, 1, &kWarpKernelSource, nullptr, &status);
        utility::checkStatus(status, "clCreateProgramWithSource");

        status = clBuildProgram(program, 1, &device, nullptr, nullptr, nullptr);
        if(status != CL_SUCCESS) {
            std::ostringstream error;
            error << "clBuildProgram failed with OpenCL error " << status;
            const std::string buildLog = utility::getBuildLog(program, device);
            if(!buildLog.empty()) {
                error << '\n' << buildLog;
            }
            releaseOpenClObjects(false);
            throw utility::ClExecutionError(error.str());
        }

        kernel = clCreateKernel(program, "warp_perspective_linear", &status);
        utility::checkStatus(status, "clCreateKernel");
        if(!runStartupCorrectnessProbe(context, device, queue, kernel)) {
            throw utility::ClExecutionError("OpenCL startup correctness probe failed");
        }
    } catch(const utility::ClExecutionError& err) {
        releaseOpenClObjects(false);
        return false;
    }
    planeData.clear();
    planeData.resize(numPlanes);
    initialized = true;
    return true;
}

bool WarpGPUMat::buildPlane(int plane, const MatConfig& config) {
    validateConfig(config);
    const auto planeIndex = utility::validatePlaneIndex(plane, planeData.size(), "WarpGPU plane index must be non-negative", "WarpGPU plane index out of bounds");
    utility::requireInitialized(initialized, "WarpGPU must be initialized");

    auto& data = planeData[planeIndex];
    if(data.enqueued) {
        throw std::runtime_error("WarpGPU::buildPlane does not support rebuilding a plane before WarpGPU::warp");
    }

    const bool configChanged = !(data.config == config);

    if(!configChanged && data.buffers.inverseMatrixBuffer != nullptr && data.buffers.borderValueBuffer != nullptr) {
        return true;
    }

    auto inverseMatrix = flatten<3, 3>(dai::matrix::getMatrixInverse(config.matrix));
    cl_int status = CL_SUCCESS;

    try {
        if(data.buffers.inverseMatrixBuffer == nullptr) {
            utility::validateBufferSize(device, sizeof(float) * inverseMatrix.size(), "clCreateBuffer(inverseMatrixBuffer)");
            data.buffers.inverseMatrixBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(float) * inverseMatrix.size(), nullptr, &status);
            utility::checkStatus(status, "clCreateBuffer(inverseMatrixBuffer)");
        }
        if(data.buffers.borderValueBuffer == nullptr) {
            utility::validateBufferSize(device, sizeof(std::uint16_t) * config.borderValue.size(), "clCreateBuffer(borderValueBuffer)");
            data.buffers.borderValueBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(std::uint16_t) * config.borderValue.size(), nullptr, &status);
            utility::checkStatus(status, "clCreateBuffer(borderValueBuffer)");
        }

        utility::checkStatus(clEnqueueWriteBuffer(
                        queue, data.buffers.inverseMatrixBuffer, CL_TRUE, 0, sizeof(float) * inverseMatrix.size(), inverseMatrix.data(), 0, nullptr, nullptr),
                    "clEnqueueWriteBuffer(inverseMatrixBuffer)");
        utility::checkStatus(clEnqueueWriteBuffer(queue,
                                         data.buffers.borderValueBuffer,
                                         CL_TRUE,
                                         0,
                                         sizeof(std::uint16_t) * config.borderValue.size(),
                                         config.borderValue.data(),
                                         0,
                                         nullptr,
                                         nullptr),
                    "clEnqueueWriteBuffer(borderValueBuffer)");
    } catch(const utility::ClExecutionError& err) {
        logError(logger, err);
        return false;
    }

    data.config = config;
    data.enqueued = false;
    return true;
}

void WarpGPUMat::enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) {
    utility::requireInitialized(initialized, "WarpGPU::init must be called before WarpGPU::warp");
    if(src == nullptr || dst == nullptr) {
        throw std::invalid_argument("WarpGPU::enqueueWarpPlane requires valid source and destination memory");
    }
    const auto planeIndex = utility::validatePlaneIndex(
        plane, planeData.size(), "WarpGPU::enqueueWarpPlane requires a valid plane index", "WarpGPU::enqueueWarpPlane requires a valid plane index");

    auto& data = planeData[planeIndex];
    const auto& config = data.config;
    if(data.buffers.inverseMatrixBuffer == nullptr || data.buffers.borderValueBuffer == nullptr) {
        throw std::runtime_error("WarpGPU::enqueueWarpPlane requires buildPlane to succeed first");
    }
    if(data.enqueued) {
        throw std::runtime_error("WarpGPU::enqueueWarpPlane does not support reusing a plane before WarpGPU::warp");
    }

    const std::size_t pixelStrideBytes = config.channels * config.bytesPerChannel;
    const std::size_t srcBytesRequired = utility::getRequiredImageBytes(config.srcWidth, config.srcHeight, config.srcStrideBytes, pixelStrideBytes);
    const std::size_t dstBytesRequired = utility::getRequiredImageBytes(config.dstWidth, config.dstHeight, config.dstStrideBytes, pixelStrideBytes);
    if(src->getOffsetData().size() < srcBytesRequired) {
        throw std::runtime_error("WarpGPU source buffer is smaller than the configured source image");
    }
    if(dst->getOffsetData().size() < dstBytesRequired) {
        throw std::runtime_error("WarpGPU destination buffer is smaller than the configured destination image");
    }

    enum class BufferRole : uint8_t { Source, Destination };

    auto bindBuffer = [&](std::unique_ptr<CachedBuffer>& zcCache,
                          std::unique_ptr<CachedBuffer>& lCache,
                          const std::shared_ptr<OffsetMemory>& memory,
                          BufferRole role,
                          cl_mem_flags flags,
                          std::size_t requiredBytes,
                          bool zeroCopyPreferred) {
        utility::DeviceBufferBinding binding;
        cl_int status = CL_SUCCESS;
        void* hostPtr = memory->getOffsetData().data();
        std::size_t bufferBytes = requiredBytes;
        cl_int offsetBytes = 0;
        int dmaBufFd = -1;
        bool usesExternalImport = false;
        std::shared_ptr<void> allocationOwner;
#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
        std::optional<utility::ExternalMemoryInfo> externalMemoryInfo;
        const bool externalImportSupported = qcomDmabufHostPtrSupported || qcomIonHostPtrSupported;
        if(zeroCopyPreferred && externalImportSupported) {
            externalMemoryInfo = utility::getExternalMemoryInfo(memory);
            if(externalMemoryInfo.has_value()) {
                hostPtr = externalMemoryInfo->hostPtr;
                bufferBytes = memory->getData().size();
                if(memory->getOffset() > static_cast<std::size_t>(std::numeric_limits<cl_int>::max())) {
                    throw std::runtime_error("WarpGPU offset exceeds OpenCL kernel argument range");
                }
                offsetBytes = static_cast<cl_int>(memory->getOffset());
                dmaBufFd = externalMemoryInfo->dmaBufFd;
                allocationOwner = externalMemoryInfo->allocationOwner;
    #if defined(CL_MEM_HOST_IOCOHERENT_QCOM)
                if(qcomIoCoherentHostPtrSupported) {
                    externalMemoryInfo->hostCachePolicy = CL_MEM_HOST_IOCOHERENT_QCOM;
                }
    #endif
                usesExternalImport = true;
            }
        }
#endif

        auto getCache = [&](bool externalImport) -> std::unique_ptr<CachedBuffer>& { return externalImport ? zcCache : lCache; };
        auto ensureCache = [&](std::unique_ptr<CachedBuffer>& cacheRef) -> CachedBuffer& {
            if(cacheRef == nullptr) {
                cacheRef = std::make_unique<CachedBuffer>();
            }
            return *cacheRef;
        };

        auto bindCache = [&](bool externalImport) {
            auto& cache = ensureCache(getCache(externalImport));
            const char* bufferLabel = role == BufferRole::Source ? "source buffer" : "destination buffer";
            const bool canReuse = cache.matches(bufferBytes, flags, externalImport, hostPtr, dmaBufFd, allocationOwner);
            if(canReuse) {
                binding.buffer = cache.buffer;
                return true;
            }

            cache.release();

#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
            if(externalImport) {
                utility::validateBufferSize(device, bufferBytes, bufferLabel);
                const cl_mem_flags importFlags = flags | CL_MEM_USE_HOST_PTR | CL_MEM_EXT_HOST_PTR_QCOM;
                if(qcomDmabufHostPtrSupported) {
    #if defined(CL_MEM_DMABUF_HOST_PTR_QCOM)
                    cl_mem_dmabuf_host_ptr dmabufHostPtr = {};
                    dmabufHostPtr.ext_host_ptr.allocation_type = CL_MEM_DMABUF_HOST_PTR_QCOM;
                    dmabufHostPtr.ext_host_ptr.host_cache_policy = externalMemoryInfo->hostCachePolicy;
                    dmabufHostPtr.dmabuf_filedesc = dmaBufFd;
                    dmabufHostPtr.dmabuf_hostptr = hostPtr;
                    cache.buffer = clCreateBuffer(context, importFlags, bufferBytes, &dmabufHostPtr, &status);
    #endif
                } else if(qcomIonHostPtrSupported) {
    #if defined(CL_MEM_ION_HOST_PTR_QCOM)
                    cl_mem_ion_host_ptr ionHostPtr = {};
                    ionHostPtr.ext_host_ptr.allocation_type = CL_MEM_ION_HOST_PTR_QCOM;
                    ionHostPtr.ext_host_ptr.host_cache_policy = externalMemoryInfo->hostCachePolicy;
                    ionHostPtr.ion_filedesc = dmaBufFd;
                    ionHostPtr.ion_hostptr = hostPtr;
                    cache.buffer = clCreateBuffer(context, importFlags, bufferBytes, &ionHostPtr, &status);
    #endif
                } else {
                    status = CL_INVALID_OPERATION;
                }
                if(status != CL_SUCCESS) {
                    if(logger != nullptr) {
                        logger->warn("ImageManip GPU zero-copy import failed with OpenCL error {}; falling back to staged copies", status);
                    }
                    cache.release();
                    return false;
                }
            } else {
#endif
                utility::validateBufferSize(device, bufferBytes, bufferLabel);
                cache.buffer = clCreateBuffer(context, flags, bufferBytes, nullptr, &status);
                utility::checkStatus(status, "clCreateBuffer");
#if defined(CL_MEM_EXT_HOST_PTR_QCOM) && (defined(CL_MEM_ION_HOST_PTR_QCOM) || defined(CL_MEM_DMABUF_HOST_PTR_QCOM))
            }
#endif

            cache.sizeBytes = bufferBytes;
            cache.flags = flags;
            cache.hostPtr = externalImport ? hostPtr : nullptr;
            cache.dmaBufFd = externalImport ? dmaBufFd : -1;
            cache.usesExternalImport = externalImport;
            cache.externalMemory = externalImport ? allocationOwner : nullptr;
            if(logger != nullptr) {
                logger->trace("ImageManip GPU created {} buffer as {}",
                              role == BufferRole::Source ? "source" : "destination",
                              externalImport ? "zero-copy" : "staged");
                logger->trace("Could reuse {} = {} && {} && {} && {} && ({} || {})", canReuse, cache.buffer != nullptr, cache.sizeBytes == bufferBytes, cache.flags == flags, cache.usesExternalImport == externalImport, !externalImport, cache.hostPtr == hostPtr);
            }
            binding.buffer = cache.buffer;
            return true;
        };

        if(usesExternalImport && !bindCache(true)) {
            usesExternalImport = false;
            hostPtr = memory->getOffsetData().data();
            bufferBytes = requiredBytes;
            offsetBytes = 0;
            dmaBufFd = -1;
        }
        if(!usesExternalImport) {
            bindCache(false);
        }

        binding.requiresUpload = role == BufferRole::Source && !usesExternalImport;
        binding.requiresDownload = role == BufferRole::Destination && !usesExternalImport;
        binding.offsetBytes = offsetBytes;

        return binding;
    };

    const bool zeroCopyPreferred = true;
    utility::DeviceBufferBinding srcBinding =
        bindBuffer(fullSrcBuffer, data.buffers.srcBuffer, src, BufferRole::Source, CL_MEM_READ_ONLY, srcBytesRequired, zeroCopyPreferred);
    utility::DeviceBufferBinding dstBinding =
        bindBuffer(fullDstBuffer, data.buffers.dstBuffer, dst, BufferRole::Destination, CL_MEM_WRITE_ONLY, dstBytesRequired, zeroCopyPreferred);

    if(srcBinding.requiresUpload) {
        utility::checkStatus(clEnqueueWriteBuffer(queue, srcBinding.buffer, CL_FALSE, 0, srcBytesRequired, src->getOffsetData().data(), 0, nullptr, nullptr),
                    "clEnqueueWriteBuffer");
    }

    const WarpKernelArgs kernelArgs = makeWarpKernelArgs(srcBinding.buffer,
                                                         dstBinding.buffer,
                                                         data.buffers.inverseMatrixBuffer,
                                                         data.buffers.borderValueBuffer,
                                                         srcBinding.offsetBytes,
                                                         dstBinding.offsetBytes,
                                                         config);
    setWarpKernelArgs(kernel, kernelArgs, "runtime");

    const size_t globalWorkSize[2] = {config.dstWidth, config.dstHeight};
    utility::checkStatus(clEnqueueNDRangeKernel(queue, kernel, 2, nullptr, globalWorkSize, nullptr, 0, nullptr, nullptr), "clEnqueueNDRangeKernel");

    if(dstBinding.requiresDownload) {
        utility::checkStatus(clEnqueueReadBuffer(queue, dstBinding.buffer, CL_FALSE, 0, dstBytesRequired, dst->getOffsetData().data(), 0, nullptr, nullptr),
                    "clEnqueueReadBuffer");
    }

    data.enqueued = true;
}

void WarpGPUMat::releasePendingPlanes() {
    for(auto& data : planeData) {
        data.enqueued = false;
    }
}

void WarpGPUMat::releaseOpenClObjects(bool buffersOnly) {
    for(auto& data : planeData) {
        if(data.buffers.borderValueBuffer != nullptr) {
            clReleaseMemObject(data.buffers.borderValueBuffer);
            data.buffers.borderValueBuffer = nullptr;
        }
        if(data.buffers.inverseMatrixBuffer != nullptr) {
            clReleaseMemObject(data.buffers.inverseMatrixBuffer);
            data.buffers.inverseMatrixBuffer = nullptr;
        }
        data.buffers.srcBuffer = nullptr;
        data.buffers.dstBuffer = nullptr;
    }
    fullSrcBuffer = nullptr;
    fullDstBuffer = nullptr;
    if(!buffersOnly) {
        if(kernel != nullptr) {
            clReleaseKernel(kernel);
            kernel = nullptr;
        }
        if(program != nullptr) {
            clReleaseProgram(program);
            program = nullptr;
        }
        if(queue != nullptr) {
            clReleaseCommandQueue(queue);
            queue = nullptr;
        }
        if(context != nullptr) {
            clReleaseContext(context);
            context = nullptr;
        }
        device = nullptr;
        platform = nullptr;
        qcomIoCoherentHostPtrSupported = false;
        qcomDmabufHostPtrSupported = false;
        qcomIonHostPtrSupported = false;
        initialized = false;
    }
}

}  // namespace dai::gate
