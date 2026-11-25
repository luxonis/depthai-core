#include "ImageManipGPUMesh.hpp"

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

inline void write_border_pixel(__global uchar* dstPixel, __constant const ushort* borderValue, int channels, int bytesPerChannel) {
    for(int c = 0; c < channels; ++c) {
        if(bytesPerChannel == 1) {
            dstPixel[c] = convert_uchar_sat(borderValue[c]);
        } else {
            ((__global ushort*)dstPixel)[c] = borderValue[c];
        }
    }
}

__kernel void warp_perspective_linear(
        __global const uchar* srcBytes,
        __global uchar* dstBytes,
        __global const float* meshX,
        __global const float* meshY,
        __constant const ushort* borderValue,
        int meshHeight,
        int meshWidth,
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

    __global const uchar* srcBaseBytes = srcBytes + srcOffsetBytes;
    __global uchar* dstBaseBytes = dstBytes + dstOffsetBytes;

    const int dstPixelStride = channels * bytesPerChannel;
    __global uchar* dstPixel = dstBaseBytes + y * dstStrideBytes + x * dstPixelStride;

    const float meshCoordX = dstWidth > 1 ? xf * ((float)(meshWidth - 1) / (float)(dstWidth - 1)) : 0.0f;
    const float meshCoordY = dstHeight > 1 ? yf * ((float)(meshHeight - 1) / (float)(dstHeight - 1)) : 0.0f;
    const int meshCoordX0 = (int)floor(meshCoordX);
    const int meshCoordY0 = (int)floor(meshCoordY);
    const int meshCoordX1 = min(meshCoordX0 + 1, meshWidth - 1);
    const int meshCoordY1 = min(meshCoordY0 + 1, meshHeight - 1);
    const float tx = meshCoordX - (float)meshCoordX0;
    const float ty = meshCoordY - (float)meshCoordY0;
    const float oneMinusTx = 1.0f - tx;
    const float oneMinusTy = 1.0f - ty;
    const float w00 = oneMinusTx * oneMinusTy;
    const float w10 = tx * oneMinusTy;
    const float w01 = oneMinusTx * ty;
    const float w11 = tx * ty;

    const int meshIndex00 = meshCoordY0 * meshWidth + meshCoordX0;
    const int meshIndex10 = meshCoordY0 * meshWidth + meshCoordX1;
    const int meshIndex01 = meshCoordY1 * meshWidth + meshCoordX0;
    const int meshIndex11 = meshCoordY1 * meshWidth + meshCoordX1;

    const float srcX = meshX[meshIndex00] * w00 + meshX[meshIndex10] * w10 + meshX[meshIndex01] * w01 + meshX[meshIndex11] * w11;
    const float srcY = meshY[meshIndex00] * w00 + meshY[meshIndex10] * w10 + meshY[meshIndex01] * w01 + meshY[meshIndex11] * w11;

    if(srcX <= -1.0f || srcY <= -1.0f || srcX >= (float)srcWidth || srcY >= (float)srcHeight) {
        write_border_pixel(dstPixel, borderValue, channels, bytesPerChannel);
        return;
    }

    const int x0 = (int)floor(srcX);
    const int y0 = (int)floor(srcY);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float srcTx = srcX - (float)x0;
    const float srcTy = srcY - (float)y0;
    const float srcOneMinusTx = 1.0f - srcTx;
    const float srcOneMinusTy = 1.0f - srcTy;
    const float srcW00 = srcOneMinusTx * srcOneMinusTy;
    const float srcW10 = srcTx * srcOneMinusTy;
    const float srcW01 = srcOneMinusTx * srcTy;
    const float srcW11 = srcTx * srcTy;

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
            const float value = c00 * srcW00 + c10 * srcW10 + c01 * srcW01 + c11 * srcW11;
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
            const float value = c00 * srcW00 + c10 * srcW10 + c01 * srcW01 + c11 * srcW11;
            dstShorts[c] = convert_ushort_sat_rte(value);
        }
    }
}

__kernel void warp_perspective_linear_dense_mesh(
        __global const uchar* srcBytes,
        __global uchar* dstBytes,
        __global const float* meshX,
        __global const float* meshY,
        __constant const ushort* borderValue,
        int meshHeight,
        int meshWidth,
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

    __global const uchar* srcBaseBytes = srcBytes + srcOffsetBytes;
    __global uchar* dstBaseBytes = dstBytes + dstOffsetBytes;

    const int dstPixelStride = channels * bytesPerChannel;
    __global uchar* dstPixel = dstBaseBytes + y * dstStrideBytes + x * dstPixelStride;
    const int meshIndex = y * meshWidth + x;
    const float srcX = meshX[meshIndex];
    const float srcY = meshY[meshIndex];

    if(srcX <= -1.0f || srcY <= -1.0f || srcX >= (float)srcWidth || srcY >= (float)srcHeight) {
        write_border_pixel(dstPixel, borderValue, channels, bytesPerChannel);
        return;
    }

    const int x0 = (int)floor(srcX);
    const int y0 = (int)floor(srcY);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const float srcTx = srcX - (float)x0;
    const float srcTy = srcY - (float)y0;
    const float srcOneMinusTx = 1.0f - srcTx;
    const float srcOneMinusTy = 1.0f - srcTy;
    const float srcW00 = srcOneMinusTx * srcOneMinusTy;
    const float srcW10 = srcTx * srcOneMinusTy;
    const float srcW01 = srcOneMinusTx * srcTy;
    const float srcW11 = srcTx * srcTy;

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
            const float value = c00 * srcW00 + c10 * srcW10 + c01 * srcW01 + c11 * srcW11;
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
            const float value = c00 * srcW00 + c10 * srcW10 + c01 * srcW01 + c11 * srcW11;
            dstShorts[c] = convert_ushort_sat_rte(value);
        }
    }
}
)CLC";

struct WarpKernelArgs {
    cl_mem srcBuffer = nullptr;
    cl_mem dstBuffer = nullptr;
    cl_mem meshXBuffer = nullptr;
    cl_mem meshYBuffer = nullptr;
    cl_mem borderValueBuffer = nullptr;
    cl_int meshHeight = 0;
    cl_int meshWidth = 0;
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

// Keep mesh allocations within OpenCV's practical signed-int-backed allocation range so we fail before downstream map allocation does.
constexpr std::size_t kMaxMeshAllocationBytes = static_cast<std::size_t>(std::numeric_limits<int>::max());

void validateConfig(const MeshConfig& config) {
    if(config.srcWidth == 0 || config.srcHeight == 0 || config.dstWidth == 0 || config.dstHeight == 0) {
        throw std::invalid_argument("WarpGPU dimensions must be non-zero");
    }
    if(config.meshX == nullptr || config.meshY == nullptr) {
        throw std::invalid_argument("WarpGPU mesh must be initialized");
    }
    if(config.meshWidth < 2 || config.meshHeight < 2) {
        throw std::invalid_argument("WarpGPU mesh dimensions must be at least 2x2");
    }
    if(config.meshWidth > config.dstWidth || config.meshHeight > config.dstHeight) {
        throw std::invalid_argument("WarpGPU mesh dimensions must not exceed destination dimensions");
    }
    const std::size_t meshPointCount = config.meshWidth * config.meshHeight;
    if(meshPointCount > kMaxMeshAllocationBytes / sizeof(float)) {
        throw std::invalid_argument("WarpGPU mesh is too large");
    }
    if(config.meshX->size() != meshPointCount || config.meshY->size() != meshPointCount) {
        throw std::invalid_argument("WarpGPU mesh size does not match mesh dimensions");
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
                                  cl_mem meshXBuffer,
                                  cl_mem meshYBuffer,
                                  cl_mem borderValueBuffer,
                                  cl_int meshHeight,
                                  cl_int meshWidth,
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
    args.meshXBuffer = meshXBuffer;
    args.meshYBuffer = meshYBuffer;
    args.borderValueBuffer = borderValueBuffer;
    args.meshHeight = meshHeight;
    args.meshWidth = meshWidth;
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
                                  cl_mem meshXBuffer,
                                  cl_mem meshYBuffer,
                                  cl_mem borderValueBuffer,
                                  cl_int srcOffsetBytes,
                                  cl_int dstOffsetBytes,
                                  const MeshConfig& config) {
    return makeWarpKernelArgs(srcBuffer,
                              dstBuffer,
                              meshXBuffer,
                              meshYBuffer,
                              borderValueBuffer,
                              static_cast<cl_int>(config.meshHeight),
                              static_cast<cl_int>(config.meshWidth),
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
    setArg(2, sizeof(cl_mem), &args.meshXBuffer, "meshX");
    setArg(3, sizeof(cl_mem), &args.meshYBuffer, "meshY");
    setArg(4, sizeof(cl_mem), &args.borderValueBuffer, "borderValue");
    setArg(5, sizeof(cl_int), &args.meshHeight, "meshHeight");
    setArg(6, sizeof(cl_int), &args.meshWidth, "meshWidth");
    setArg(7, sizeof(cl_int), &args.srcOffsetBytes, "srcOffsetBytes");
    setArg(8, sizeof(cl_int), &args.dstOffsetBytes, "dstOffsetBytes");
    setArg(9, sizeof(cl_int), &args.srcWidth, "srcWidth");
    setArg(10, sizeof(cl_int), &args.srcHeight, "srcHeight");
    setArg(11, sizeof(cl_int), &args.srcStrideBytes, "srcStrideBytes");
    setArg(12, sizeof(cl_int), &args.dstWidth, "dstWidth");
    setArg(13, sizeof(cl_int), &args.dstHeight, "dstHeight");
    setArg(14, sizeof(cl_int), &args.dstStrideBytes, "dstStrideBytes");
    setArg(15, sizeof(cl_int), &args.channels, "channels");
    setArg(16, sizeof(cl_int), &args.bytesPerChannel, "bytesPerChannel");
}

bool runStartupCorrectnessProbe(cl_context context, cl_device_id device, cl_command_queue queue, cl_kernel kernel, bool denseMesh) {
    constexpr cl_int width = 5;
    constexpr cl_int height = 4;
    constexpr cl_int strideBytes = width;
    constexpr cl_int channels = 1;
    constexpr cl_int bytesPerChannel = 1;
    constexpr size_t bufferSize = static_cast<size_t>(width * height);
    const cl_int meshWidth = denseMesh ? width : 2;
    const cl_int meshHeight = denseMesh ? height : 2;
    const size_t globalWorkSize[2] = {static_cast<size_t>(width), static_cast<size_t>(height)};
    std::vector<float> meshX;
    std::vector<float> meshY;
    const std::size_t meshPointCount = static_cast<std::size_t>(meshWidth * meshHeight);
    meshX.reserve(meshPointCount);
    meshY.reserve(meshPointCount);
    for(cl_int y = 0; y < meshHeight; ++y) {
        const float srcY = meshHeight > 1 ? static_cast<float>(y) * static_cast<float>(height - 1) / static_cast<float>(meshHeight - 1) : 0.0f;
        for(cl_int x = 0; x < meshWidth; ++x) {
            const float srcX = meshWidth > 1 ? static_cast<float>(x) * static_cast<float>(width - 1) / static_cast<float>(meshWidth - 1) : 0.0f;
            meshX.push_back(srcX);
            meshY.push_back(srcY);
        }
    }
    const std::array<std::uint16_t, 4> borderValue = {0, 0, 0, 0};
    const std::array<std::uint8_t, bufferSize> srcData = {1, 17, 33, 49, 65, 81, 97, 113, 129, 145, 161, 177, 193, 209, 225, 241, 13, 29, 45, 61};
    std::array<std::uint8_t, bufferSize> dstData{};

    cl_mem srcBuffer = nullptr;
    cl_mem dstBuffer = nullptr;
    cl_mem meshXBuffer = nullptr;
    cl_mem meshYBuffer = nullptr;
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
        utility::validateBufferSize(device, meshX.size() * sizeof(meshX[0]), "clCreateBuffer(startupProbeMeshX)");
        meshXBuffer = clCreateBuffer(
            context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, meshX.size() * sizeof(meshX[0]), const_cast<float*>(meshX.data()), &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeMeshX)");
        utility::validateBufferSize(device, meshY.size() * sizeof(meshY[0]), "clCreateBuffer(startupProbeMeshY)");
        meshYBuffer = clCreateBuffer(
            context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, meshY.size() * sizeof(meshY[0]), const_cast<float*>(meshY.data()), &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeMeshY)");
        utility::validateBufferSize(device, borderValue.size() * sizeof(borderValue[0]), "clCreateBuffer(startupProbeBorderValue)");
        borderValueBuffer = clCreateBuffer(context,
                                           CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                                           borderValue.size() * sizeof(borderValue[0]),
                                           const_cast<std::uint16_t*>(borderValue.data()),
                                           &status);
        utility::checkStatus(status, "clCreateBuffer(startupProbeBorderValue)");

        const WarpKernelArgs kernelArgs = makeWarpKernelArgs(srcBuffer,
                                                             dstBuffer,
                                                             meshXBuffer,
                                                             meshYBuffer,
                                                             borderValueBuffer,
                                                             meshHeight,
                                                             meshWidth,
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
        releaseBuffer(meshYBuffer);
        releaseBuffer(meshXBuffer);
        releaseBuffer(dstBuffer);
        releaseBuffer(srcBuffer);
        throw;
    }

    releaseBuffer(borderValueBuffer);
    releaseBuffer(meshYBuffer);
    releaseBuffer(meshXBuffer);
    releaseBuffer(dstBuffer);
    releaseBuffer(srcBuffer);

    return dstData == srcData;
}

}  // namespace

WarpGPUMesh::~WarpGPUMesh() {
    releaseOpenClObjects(false);
}

std::vector<Point2f> WarpGPUMesh::getMeshFromMatrix(const std::array<std::array<float, 3>, 3>& matrix,
                                                    std::size_t meshWidth,
                                                    std::size_t meshHeight,
                                                    std::size_t dstWidth,
                                                    std::size_t dstHeight) {
    if(meshWidth < 2 || meshHeight < 2) {
        throw std::invalid_argument("WarpGPU mesh dimensions must be at least 2x2");
    }
    if(dstWidth == 0 || dstHeight == 0) {
        throw std::invalid_argument("WarpGPU destination dimensions must be non-zero");
    }
    if(meshWidth > dstWidth || meshHeight > dstHeight) {
        throw std::invalid_argument("WarpGPU mesh dimensions must not exceed destination dimensions");
    }
    const std::size_t meshPointCount = meshWidth * meshHeight;
    if(meshPointCount > kMaxMeshAllocationBytes / sizeof(Point2f)) {
        throw std::invalid_argument("WarpGPU mesh is too large");
    }

    const auto inverseMatrix = dai::matrix::getMatrixInverse(matrix);
    std::vector<Point2f> mesh;
    mesh.reserve(meshPointCount);

    for(std::size_t y = 0; y < meshHeight; ++y) {
        const float dstY = meshHeight > 1 ? static_cast<float>(y) * static_cast<float>(dstHeight - 1) / static_cast<float>(meshHeight - 1) : 0.0f;
        for(std::size_t x = 0; x < meshWidth; ++x) {
            const float dstX =
                meshWidth > 1 ? static_cast<float>(x) * static_cast<float>(dstWidth - 1) / static_cast<float>(meshWidth - 1) : 0.0f;

            const float denom = inverseMatrix[2][0] * dstX + inverseMatrix[2][1] * dstY + inverseMatrix[2][2];
            if(std::fabs(denom) < 1.0e-8f) {
                mesh.emplace_back(-2.0f, -2.0f);
                continue;
            }

            const float srcX = (inverseMatrix[0][0] * dstX + inverseMatrix[0][1] * dstY + inverseMatrix[0][2]) / denom;
            const float srcY = (inverseMatrix[1][0] * dstX + inverseMatrix[1][1] * dstY + inverseMatrix[1][2]) / denom;
            mesh.emplace_back(srcX, srcY);
        }
    }

    return mesh;
}

bool WarpGPUMesh::init(int numPlanes) {
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
        denseKernel = clCreateKernel(program, "warp_perspective_linear_dense_mesh", &status);
        utility::checkStatus(status, "clCreateKernel(dense)");
        if(!runStartupCorrectnessProbe(context, device, queue, kernel, false)) {
            throw utility::ClExecutionError("OpenCL startup correctness probe failed");
        }
        if(!runStartupCorrectnessProbe(context, device, queue, denseKernel, true)) {
            throw utility::ClExecutionError("OpenCL dense-mesh startup correctness probe failed");
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

bool WarpGPUMesh::buildPlane(int plane, const MeshConfig& config) {
    validateConfig(config);
    const auto planeIndex = utility::validatePlaneIndex(plane, planeData.size(), "WarpGPU plane index must be non-negative", "WarpGPU plane index out of bounds");
    utility::requireInitialized(initialized, "WarpGPU must be initialized");

    auto& data = planeData[planeIndex];
    if(data.enqueued) {
        throw std::runtime_error("WarpGPU::buildPlane does not support rebuilding a plane before WarpGPU::warp");
    }

    const bool configChanged = !(data.config == config);

    if(!configChanged && data.buffers.meshXBuffer != nullptr && data.buffers.meshYBuffer != nullptr && data.buffers.borderValueBuffer != nullptr) {
        return true;
    }

    const auto& meshXData = *config.meshX;
    const auto& meshYData = *config.meshY;
    cl_int status = CL_SUCCESS;

    try {
        if(data.buffers.meshXBuffer != nullptr) {
            clReleaseMemObject(data.buffers.meshXBuffer);
            data.buffers.meshXBuffer = nullptr;
        }
        if(data.buffers.meshYBuffer != nullptr) {
            clReleaseMemObject(data.buffers.meshYBuffer);
            data.buffers.meshYBuffer = nullptr;
        }
        utility::validateBufferSize(device, sizeof(float) * meshXData.size(), "clCreateBuffer(meshXBuffer)");
        data.buffers.meshXBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(float) * meshXData.size(), nullptr, &status);
        utility::checkStatus(status, "clCreateBuffer(meshXBuffer)");
        utility::validateBufferSize(device, sizeof(float) * meshYData.size(), "clCreateBuffer(meshYBuffer)");
        data.buffers.meshYBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(float) * meshYData.size(), nullptr, &status);
        utility::checkStatus(status, "clCreateBuffer(meshYBuffer)");
        if(data.buffers.borderValueBuffer == nullptr) {
            utility::validateBufferSize(device, sizeof(std::uint16_t) * config.borderValue.size(), "clCreateBuffer(borderValueBuffer)");
            data.buffers.borderValueBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(std::uint16_t) * config.borderValue.size(), nullptr, &status);
            utility::checkStatus(status, "clCreateBuffer(borderValueBuffer)");
        }

        utility::checkStatus(
            clEnqueueWriteBuffer(queue, data.buffers.meshXBuffer, CL_TRUE, 0, sizeof(float) * meshXData.size(), meshXData.data(), 0, nullptr, nullptr),
            "clEnqueueWriteBuffer(meshXBuffer)");
        utility::checkStatus(
            clEnqueueWriteBuffer(queue, data.buffers.meshYBuffer, CL_TRUE, 0, sizeof(float) * meshYData.size(), meshYData.data(), 0, nullptr, nullptr),
            "clEnqueueWriteBuffer(meshYBuffer)");
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

void WarpGPUMesh::enqueueWarpPlane(int plane, const std::shared_ptr<OffsetMemory>& src, const std::shared_ptr<OffsetMemory>& dst) {
    utility::requireInitialized(initialized, "WarpGPU::init must be called before WarpGPU::warp");
    if(src == nullptr || dst == nullptr) {
        throw std::invalid_argument("WarpGPU::enqueueWarpPlane requires valid source and destination memory");
    }
    const auto planeIndex = utility::validatePlaneIndex(
        plane, planeData.size(), "WarpGPU::enqueueWarpPlane requires a valid plane index", "WarpGPU::enqueueWarpPlane requires a valid plane index");

    auto& data = planeData[planeIndex];
    const auto& config = data.config;
    if(data.buffers.meshXBuffer == nullptr || data.buffers.meshYBuffer == nullptr || data.buffers.borderValueBuffer == nullptr) {
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
                                                         data.buffers.meshXBuffer,
                                                         data.buffers.meshYBuffer,
                                                         data.buffers.borderValueBuffer,
                                                         srcBinding.offsetBytes,
                                                         dstBinding.offsetBytes,
                                                         config);
    const bool denseMesh = config.meshWidth == config.dstWidth && config.meshHeight == config.dstHeight;
    cl_kernel selectedKernel = denseMesh ? denseKernel : kernel;
    setWarpKernelArgs(selectedKernel, kernelArgs, denseMesh ? "runtimeDense" : "runtime");

    const size_t globalWorkSize[2] = {config.dstWidth, config.dstHeight};
    utility::checkStatus(clEnqueueNDRangeKernel(queue, selectedKernel, 2, nullptr, globalWorkSize, nullptr, 0, nullptr, nullptr), "clEnqueueNDRangeKernel");

    if(dstBinding.requiresDownload) {
        utility::checkStatus(clEnqueueReadBuffer(queue, dstBinding.buffer, CL_FALSE, 0, dstBytesRequired, dst->getOffsetData().data(), 0, nullptr, nullptr),
                    "clEnqueueReadBuffer");
    }

    data.enqueued = true;
}

void WarpGPUMesh::releasePendingPlanes() {
    for(auto& data : planeData) {
        data.enqueued = false;
    }
}

void WarpGPUMesh::releaseOpenClObjects(bool buffersOnly) {
    for(auto& data : planeData) {
        if(data.buffers.borderValueBuffer != nullptr) {
            clReleaseMemObject(data.buffers.borderValueBuffer);
            data.buffers.borderValueBuffer = nullptr;
        }
        if(data.buffers.meshYBuffer != nullptr) {
            clReleaseMemObject(data.buffers.meshYBuffer);
            data.buffers.meshYBuffer = nullptr;
        }
        if(data.buffers.meshXBuffer != nullptr) {
            clReleaseMemObject(data.buffers.meshXBuffer);
            data.buffers.meshXBuffer = nullptr;
        }
        data.buffers.srcBuffer = nullptr;
        data.buffers.dstBuffer = nullptr;
    }
    fullSrcBuffer = nullptr;
    fullDstBuffer = nullptr;
    if(!buffersOnly) {
        if(denseKernel != nullptr) {
            clReleaseKernel(denseKernel);
            denseKernel = nullptr;
        }
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
